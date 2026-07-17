#include "controller/services.hpp"

#include "controller/ecal_io.hpp"

#include "control/balance.hpp"
#include "control/chassis_fsm.hpp"
#include "control/imu_fusion.hpp"
#include "control/leg.hpp"
#include "control/math.hpp"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace controller
{

namespace
{

void sleep_until_tick(std::chrono::steady_clock::time_point& next, float hz)
{
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(hz)));
    std::this_thread::sleep_until(next);
}

void publish_ins(const control::imu_attitude_t* att, const control::msg_raw_state_t& raw)
{
    control::msg_ins_t ins{};
    if (att != nullptr)
    {
        for (int i = 0; i < 4; ++i)
        {
            ins.quaternion[i] = att->q[i];
        }
        ins.roll = att->roll;
        ins.pitch = att->pitch;
        ins.yaw = att->yaw;
        ins.total_yaw = att->total_yaw;
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            ins.quaternion[i] = raw.quat_gt[i];
        }
        control::quat_to_euler(ins.quaternion, ins.roll, ins.pitch, ins.yaw);
        ins.total_yaw = ins.yaw;
    }
    ins.gyro_r = raw.gyro[0];
    ins.gyro_p = raw.gyro[1];
    ins.gyro_y = raw.gyro[2];
    for (int i = 0; i < 3; ++i)
    {
        ins.accel[i] = raw.accel[i];
    }
    msg::publish(ins, {false});
}

void fill_leg_log(control::msg_log_t& log, const control::link_solver& leg, bool left)
{
    if (left)
    {
        log.l_len = leg.len_;
        log.l_dlen = leg.dlen_;
        log.l_alpha = leg.alpha_;
        log.l_dalpha = leg.dalpha_;
        log.l_phi = leg.phi_;
        log.l_dphi = leg.dphi_;
        log.l_alpha_eq = leg.alpha_eq_;
        log.l_t_hip = leg.treal_hip_;
        log.l_total_phi = leg.total_phi_;
        log.l_n = leg.n_;
        log.l_f = leg.freal_;
        log.l_fs = leg.fs_;
        log.l_flat = leg.flat_ ? 1.0f : 0.0f;
        log.l_neutral = leg.neutral_ ? 1.0f : 0.0f;
        return;
    }

    log.r_len = leg.len_;
    log.r_dlen = leg.dlen_;
    log.r_alpha = leg.alpha_;
    log.r_dalpha = leg.dalpha_;
    log.r_phi = leg.phi_;
    log.r_dphi = leg.dphi_;
    log.r_alpha_eq = leg.alpha_eq_;
    log.r_t_hip = leg.treal_hip_;
    log.r_total_phi = leg.total_phi_;
    log.r_n = leg.n_;
    log.r_f = leg.freal_;
    log.r_fs = leg.fs_;
    log.r_flat = leg.flat_ ? 1.0f : 0.0f;
    log.r_neutral = leg.neutral_ ? 1.0f : 0.0f;
}

control::msg_log_t make_log(double time, const control::msg_ins_t& ins, const control::msg_raw_state_t& raw,
                            const control::link_solver& left, const control::link_solver& right,
                            const control::msg_ctrl_t& ctrl, const control::msg_motor_cmd_t& motor_cmd,
                            const control::msg_cmd_t& cmd, control::chassis_state fsm, float n_total, float x,
                            float v, float az)
{
    control::msg_log_t log{};
    log.time = time;
    std::memcpy(log.quaternion, ins.quaternion, sizeof(log.quaternion));
    log.roll = ins.roll;
    log.pitch = ins.pitch;
    log.yaw = ins.yaw;
    log.gyro[0] = ins.gyro_r;
    log.gyro[1] = ins.gyro_p;
    log.gyro[2] = ins.gyro_y;
    std::memcpy(log.accel, ins.accel, sizeof(log.accel));
    log.x = x;
    log.v = v;
    log.az = az;
    fill_leg_log(log, left, true);
    fill_leg_log(log, right, false);
    for (int i = 0; i < 6; ++i)
    {
        log.m_q[i] = raw.motors[i].q;
        log.m_dq[i] = raw.motors[i].dq;
        log.m_tau[i] = raw.motors[i].tau;
        log.cmd_tau[i] = motor_cmd.tau[i];
    }
    log.Tl0 = ctrl.Tl[0];
    log.Tl1 = ctrl.Tl[1];
    log.Tr0 = ctrl.Tr[0];
    log.Tr1 = ctrl.Tr[1];
    log.Twl = ctrl.Twl;
    log.Twr = ctrl.Twr;
    log.fsm = static_cast<std::uint8_t>(fsm);
    log.n_total = n_total;
    log.cmd_v = cmd.v;
    log.cmd_x = cmd.x;
    log.cmd_len = cmd.len;
    log.cmd_yaw = cmd.yaw;
    log.cmd_dyaw = cmd.dyaw;
    log.cmd_move = cmd.move;
    return log;
}

void print_state_block(const control::msg_log_t& log)
{
    std::printf("[t=%.3f] imu quat=(%.3f %.3f %.3f %.3f) rpy_rad=(%.4f %.4f %.4f) gyro=(%.3f %.3f %.3f) "
                "accel=(%.3f %.3f %.3f)\n",
                log.time, log.quaternion[0], log.quaternion[1], log.quaternion[2], log.quaternion[3], log.roll,
                log.pitch, log.yaw, log.gyro[0], log.gyro[1], log.gyro[2], log.accel[0], log.accel[1],
                log.accel[2]);

    auto print_motor = [&](const char* name, int i) {
        std::printf("  %s: q=%.4f dq=%.4f tau_est=%.4f tau_cmd=%.4f\n", name, log.m_q[i], log.m_dq[i],
                    log.m_tau[i], log.cmd_tau[i]);
    };
    auto print_link = [&](const char* tag, bool left, int motor_base) {
        const float len = left ? log.l_len : log.r_len;
        const float dlen = left ? log.l_dlen : log.r_dlen;
        const float phi = left ? log.l_phi : log.r_phi;
        const float dphi = left ? log.l_dphi : log.r_dphi;
        const float alpha = left ? log.l_alpha : log.r_alpha;
        const float dalpha = left ? log.l_dalpha : log.r_dalpha;
        const float alpha_eq = left ? log.l_alpha_eq : log.r_alpha_eq;
        const float n = left ? log.l_n : log.r_n;
        const float fs = left ? log.l_fs : log.r_fs;
        const float f_est = left ? log.l_f : log.r_f;
        const float t_hip = left ? log.l_t_hip : log.r_t_hip;
        const float total_phi = left ? log.l_total_phi : log.r_total_phi;
        const float flat = left ? log.l_flat : log.r_flat;
        const float neutral = left ? log.l_neutral : log.r_neutral;
        const float f0 = left ? log.Tl0 : log.Tr0;
        const float f1 = left ? log.Tl1 : log.Tr1;
        std::printf("  %s_link: len=%.4f dlen=%.4f phi=%.4f dphi=%.4f alpha=%.4f dalpha=%.4f alpha_eq=%.4f "
                    "n=%.4f Fs=%.4f F=(%.4f,%.4f) tau=(%.4f,%.4f,%.4f) F_est=%.4f tau_hip_est=%.4f "
                    "total_phi=%.4f flat=%d neutral=%d\n",
                    tag, len, dlen, phi, dphi, alpha, dalpha, alpha_eq, n, fs, f0, f1,
                    log.cmd_tau[motor_base], log.cmd_tau[motor_base + 1], log.cmd_tau[motor_base + 2],
                    f_est, t_hip, total_phi, static_cast<int>(flat), static_cast<int>(neutral));
    };

    print_motor("ljoint1", 0);
    print_motor("ljoint4", 1);
    print_motor("lwheel", 2);
    print_link("left", true, 0);
    print_motor("rjoint1", 3);
    print_motor("rjoint4", 4);
    print_motor("rwheel", 5);
    print_link("right", false, 3);
    std::printf("  state: fsm=%u cmd.len=%.4f cmd.x=%.4f cmd.v=%.4f cmd.yaw=%.4f cmd.dyaw=%.4f Tw=(%.4f,%.4f) move=%d n_total=%.4f "
                "odom=(x=%.4f v=%.4f az=%.4f)\n",
                static_cast<unsigned>(log.fsm), log.cmd_len, log.cmd_x, log.cmd_v, log.cmd_yaw, log.cmd_dyaw,
                log.Twl, log.Twr,
                static_cast<int>(log.cmd_move), log.n_total, log.x, log.v, log.az);
    std::fflush(stdout);
}

constexpr int k_visualizer_port = 2000;

const char* visualizer_html()
{
    return R"HTML(<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WBR observer</title>
<style>
:root{color-scheme:dark;--bg:#111418;--panel:#181d23;--grid:#303844;--text:#e7edf5;--muted:#9aa7b7;--a:#5eead4;--b:#fbbf24;--c:#93c5fd;--d:#f472b6}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px/1.4 system-ui,-apple-system,Segoe UI,sans-serif}
header{height:44px;display:flex;align-items:center;gap:18px;padding:0 14px;border-bottom:1px solid #27303a;background:#0e1115;position:sticky;top:0;z-index:1}
h1{font-size:15px;margin:0;font-weight:650}.status{color:var(--muted)}main{display:grid;grid-template-columns:repeat(2,minmax(360px,1fr));gap:10px;padding:10px}
section{background:var(--panel);border:1px solid #27303a;border-radius:8px;min-width:0}h2{font-size:13px;margin:0;padding:9px 10px;border-bottom:1px solid #27303a;font-weight:650}
canvas{display:block;width:100%;height:220px}.readout{display:flex;flex-wrap:wrap;gap:8px;padding:8px 10px;color:var(--muted);font-variant-numeric:tabular-nums}
.tag{white-space:nowrap}.a{color:var(--a)}.b{color:var(--b)}.c{color:var(--c)}.d{color:var(--d)}
@media(max-width:820px){main{grid-template-columns:1fr}canvas{height:190px}}
</style>
</head>
<body>
<header><h1>WBR observer</h1><div class="status" id="status">connecting...</div><div class="status" id="time"></div></header>
<main id="grid"></main>
<script>
const groups=[
 {id:'att',title:'Attitude',series:[['pitch','pitch','a'],['l_alpha','l alpha','b'],['r_alpha','r alpha','c'],['gyro_p','gyro p','d']]},
 {id:'cmd',title:'Command / Odom',series:[['cmd_x','cmd x','a'],['x','odom x','b'],['cmd_v','cmd v','c'],['v','odom v','d']]},
 {id:'torque',title:'Wheel Torque',series:[['Twl','Twl','a'],['Twr','Twr','b'],['cmd_tau2','cmd lwheel','c'],['cmd_tau5','cmd rwheel','d']]},
 {id:'leg',title:'Leg Geometry',series:[['l_phi','l phi','a'],['r_phi','r phi','b'],['l_len','l len','c'],['r_len','r len','d']]},
 {id:'contact',title:'Contact / Vertical',series:[['l_n','left n','a'],['r_n','right n','b'],['n_total','n total','c'],['az','az','d']]},
 {id:'motor',title:'Actual Motor Torque',series:[['m_tau0','lj1','a'],['m_tau1','lj4','b'],['m_tau3','rj1','c'],['m_tau4','rj4','d']]}
];
const css=getComputedStyle(document.documentElement);const colors={a:css.getPropertyValue('--a'),b:css.getPropertyValue('--b'),c:css.getPropertyValue('--c'),d:css.getPropertyValue('--d')};
const N=420, data=[];const grid=document.getElementById('grid');
for(const g of groups){const s=document.createElement('section');s.innerHTML=`<h2>${g.title}</h2><canvas id="${g.id}"></canvas><div class="readout" id="${g.id}r"></div>`;grid.appendChild(s);g.canvas=s.querySelector('canvas');g.ctx=g.canvas.getContext('2d');}
function resize(c){const r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=Math.max(1,r.width*d);c.height=Math.max(1,r.height*d);}
function val(o,k){return Number(o[k]??0)}
function draw(g){const c=g.canvas,ctx=g.ctx;resize(c);const w=c.width,h=c.height,p=28;ctx.clearRect(0,0,w,h);ctx.strokeStyle='#303844';ctx.lineWidth=1;for(let i=0;i<5;i++){let y=p+(h-2*p)*i/4;ctx.beginPath();ctx.moveTo(p,y);ctx.lineTo(w-8,y);ctx.stroke();}
 let vals=[];for(const [k] of g.series){for(const d of data)vals.push(val(d,k));}let mn=Math.min(...vals,0),mx=Math.max(...vals,0);if(!isFinite(mn)||mx-mn<1e-6){mn=-1;mx=1}const pad=(mx-mn)*.08;mn-=pad;mx+=pad;
 ctx.fillStyle='#9aa7b7';ctx.font=`${11*(devicePixelRatio||1)}px system-ui`;ctx.fillText(mx.toFixed(2),4,p);ctx.fillText(mn.toFixed(2),4,h-p);
 for(const [k,label,cl] of g.series){ctx.strokeStyle=colors[cl];ctx.lineWidth=2*(devicePixelRatio||1);ctx.beginPath();data.forEach((d,i)=>{const x=p+(w-p-8)*i/Math.max(1,N-1);const y=h-p-(val(d,k)-mn)*(h-2*p)/(mx-mn);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();}
 document.getElementById(g.id+'r').innerHTML=g.series.map(([k,l,cl])=>`<span class="tag ${cl}">${l}: ${val(data.at(-1)||{},k).toFixed(4)}</span>`).join('');
}
function redraw(){for(const g of groups)draw(g)}
const es=new EventSource('/events');es.onopen=()=>status.textContent='live on :2000';es.onerror=()=>status.textContent='disconnected';
es.onmessage=e=>{const o=JSON.parse(e.data);data.push(o);while(data.length>N)data.shift();time.textContent=`t=${Number(o.time||0).toFixed(3)} fsm=${o.fsm} move=${o.cmd_move}`;redraw();}
addEventListener('resize',redraw);
</script>
</body>
</html>)HTML";
}

std::string log_to_json(const control::msg_log_t& log)
{
    char buf[4096];
    std::snprintf(buf, sizeof(buf),
                  "{\"time\":%.6f,\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f,"
                  "\"gyro_p\":%.6f,\"accel0\":%.6f,\"accel2\":%.6f,"
                  "\"x\":%.6f,\"v\":%.6f,\"az\":%.6f,"
                  "\"l_len\":%.6f,\"l_dlen\":%.6f,\"l_alpha\":%.6f,\"l_dalpha\":%.6f,\"l_phi\":%.6f,\"l_dphi\":%.6f,\"l_n\":%.6f,"
                  "\"r_len\":%.6f,\"r_dlen\":%.6f,\"r_alpha\":%.6f,\"r_dalpha\":%.6f,\"r_phi\":%.6f,\"r_dphi\":%.6f,\"r_n\":%.6f,"
                  "\"n_total\":%.6f,\"Tl0\":%.6f,\"Tl1\":%.6f,\"Tr0\":%.6f,\"Tr1\":%.6f,\"Twl\":%.6f,\"Twr\":%.6f,"
                  "\"m_tau0\":%.6f,\"m_tau1\":%.6f,\"m_tau2\":%.6f,\"m_tau3\":%.6f,\"m_tau4\":%.6f,\"m_tau5\":%.6f,"
                  "\"cmd_tau0\":%.6f,\"cmd_tau1\":%.6f,\"cmd_tau2\":%.6f,\"cmd_tau3\":%.6f,\"cmd_tau4\":%.6f,\"cmd_tau5\":%.6f,"
                  "\"cmd_x\":%.6f,\"cmd_v\":%.6f,\"cmd_len\":%.6f,\"cmd_yaw\":%.6f,\"cmd_dyaw\":%.6f,\"cmd_move\":%d,\"fsm\":%u}",
                  log.time, log.roll, log.pitch, log.yaw, log.gyro[1], log.accel[0], log.accel[2], log.x, log.v,
                  log.az, log.l_len, log.l_dlen, log.l_alpha, log.l_dalpha, log.l_phi, log.l_dphi, log.l_n,
                  log.r_len, log.r_dlen, log.r_alpha, log.r_dalpha, log.r_phi, log.r_dphi, log.r_n, log.n_total,
                  log.Tl0, log.Tl1, log.Tr0, log.Tr1, log.Twl, log.Twr, log.m_tau[0], log.m_tau[1],
                  log.m_tau[2], log.m_tau[3], log.m_tau[4], log.m_tau[5], log.cmd_tau[0], log.cmd_tau[1],
                  log.cmd_tau[2], log.cmd_tau[3], log.cmd_tau[4], log.cmd_tau[5], log.cmd_x, log.cmd_v,
                  log.cmd_len, log.cmd_yaw, log.cmd_dyaw, log.cmd_move ? 1 : 0, static_cast<unsigned>(log.fsm));
    return buf;
}

void set_nonblocking(int fd)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
    {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

bool send_text(int fd, const std::string& text)
{
    const char* ptr = text.c_str();
    std::size_t left = text.size();
    while (left > 0)
    {
        const ssize_t sent = send(fd, ptr, left, MSG_NOSIGNAL);
        if (sent < 0)
        {
            return errno == EAGAIN || errno == EWOULDBLOCK;
        }
        ptr += sent;
        left -= static_cast<std::size_t>(sent);
    }
    return true;
}

}  // namespace

actuator_service::actuator_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
    io_.update_motor_cmd({});
}

actuator_service::~actuator_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void actuator_service::loop()
{
    const auto tick_wait = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(cfg_.control_hz)));
    while (running_.load())
    {
        io_.poll();

        control::msg_motor_cmd_t latest{};
        if (msg::read(sub_motor_cmd_, latest) == msg::status::ok)
        {
            motor_cmd_ = latest;
        }
        io_.update_motor_cmd(motor_cmd_);

        io_.wait_for_tick_and_commit(tick_wait);
    }
}

ins_service::ins_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
}

ins_service::~ins_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void ins_service::loop()
{
    control::mahony_filter mahony;
    mahony.reset();

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::msg_raw_state_t raw{};
        if (msg::read(sub_raw_state_, raw) == msg::status::ok)
        {
            io_.apply_imu_noise(raw);
            if (cfg_.imu_mode == control::imu_mode::bypass)
            {
                publish_ins(nullptr, raw);
            }
            else
            {
                mahony.update(raw.gyro[0], raw.gyro[1], raw.gyro[2], raw.accel[0], raw.accel[1], raw.accel[2],
                              dt);
                publish_ins(&mahony.attitude, raw);
            }
        }
        sleep_until_tick(next, cfg_.control_hz);
    }
}

command_service::command_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

command_service::~command_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void command_service::loop()
{
    control::command_fusion fusion;
    fusion.reset(cfg_.chassis);

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::input_snapshot_t latest{};
        if (msg::read(sub_input_, latest) == msg::status::ok)
        {
            input_ = latest;
        }

        control::msg_pendulum_t pendulum{};
        control::msg_ins_t ins{};
        msg::read(sub_pendulum_, pendulum);
        msg::read(sub_ins_, ins);

        fusion.update(input_, pendulum, ins, cfg_.chassis, dt);
        msg::publish(fusion.msg(), {false});
        sleep_until_tick(next, cfg_.control_hz);
    }
}

chassis_service::chassis_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

chassis_service::~chassis_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void chassis_service::loop()
{
    // Left-view positive: left leg is the reference frame, right leg is mirrored.
    control::leg_controller left(true, 0, 1, cfg_.chassis);
    control::leg_controller right(false, 3, 4, cfg_.chassis);
    control::odometry odom;
    control::chassis_fsm fsm;
    fsm.init(cfg_.chassis);

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::msg_raw_state_t raw{};
        control::msg_ins_t ins{};
        control::msg_cmd_t cmd{};
        if (msg::read(sub_raw_state_, raw) != msg::status::ok || msg::read(sub_ins_, ins) != msg::status::ok ||
            msg::read(sub_cmd_, cmd) != msg::status::ok)
        {
            sleep_until_tick(next, cfg_.control_hz);
            continue;
        }

        const float pitch = ins.pitch;
        const float dpitch = ins.gyro_p;
        const float yaw = ins.total_yaw;
        const float dyaw = ins.gyro_y;
        const float vl = raw.motors[2].dq * cfg_.chassis.rwheel * left.wheel_sign();
        const float vr = raw.motors[5].dq * cfg_.chassis.rwheel * right.wheel_sign();

        odom.update(ins.quaternion, ins.accel, (vl + vr) * 0.5f, yaw, dt);
        left.link().solve(pitch, dpitch, odom.az, raw.motors[0], raw.motors[1]);
        right.link().solve(pitch, dpitch, odom.az, raw.motors[3], raw.motors[4]);

        const auto& ll = left.link();
        const auto& rl = right.link();

        float observed_x[10] = {};
        observed_x[0] = odom.x;
        observed_x[1] = odom.v;
        observed_x[2] = yaw;
        observed_x[3] = dyaw;
        observed_x[4] = ll.alpha_;
        observed_x[5] = ll.dalpha_;
        observed_x[6] = rl.alpha_;
        observed_x[7] = rl.dalpha_;
        observed_x[8] = pitch;
        observed_x[9] = dpitch;

        control::msg_odometry_t odom_msg{};
        odom_msg.x = odom.x;
        odom_msg.v = odom.v;
        odom_msg.a_z = odom.az;
        msg::publish(odom_msg, {false});

        control::fsm_inputs fin{ins, cmd, odom_msg, left, right};
        std::memcpy(fin.observed_x, observed_x, sizeof(observed_x));
        fin.n_total = ll.n_ + rl.n_;
        fin.chassis_dead = false;

        control::fsm_outputs fout{};
        fsm.step(fin, fout);

        if (!cmd.move || cfg_.chassis.force_relax)
        {
            std::memset(&fout.motor, 0, sizeof(fout.motor));
        }

        msg::publish(fout.motor, {true});
        msg::publish(fout.pendulum, {false});
        msg::publish(make_log(raw.time, ins, raw, ll, rl, fout.ctrl, fout.motor, cmd, fsm.state(), fin.n_total,
                              odom.x, odom.v, odom.az),
                     {false});

        sleep_until_tick(next, cfg_.control_hz);
    }
}

sim_log_service::sim_log_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

sim_log_service::~sim_log_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void sim_log_service::loop()
{
    auto next = std::chrono::steady_clock::now();
    const int log_every = cfg_.logger.stdout_block && cfg_.logger.hz > 0.0f
                              ? static_cast<int>(cfg_.control_hz / cfg_.logger.hz + 0.5f)
                              : 0;
    int log_counter = 0;

    while (running_.load())
    {
        control::msg_log_t log{};
        if (log_every > 0 && msg::read(sub_log_, log) == msg::status::ok && ++log_counter >= log_every)
        {
            log_counter = 0;
            print_state_block(log);
        }
        sleep_until_tick(next, cfg_.control_hz);
    }
}

web_visualizer_service::web_visualizer_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

web_visualizer_service::~web_visualizer_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void web_visualizer_service::loop()
{
    (void)cfg_;

    const int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0)
    {
        std::fprintf(stderr, "visualizer: socket failed: %s\n", std::strerror(errno));
        return;
    }

    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(k_visualizer_port);

    if (bind(server, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || listen(server, 8) < 0)
    {
        std::fprintf(stderr, "visualizer: failed to listen on localhost:%d: %s\n", k_visualizer_port,
                     std::strerror(errno));
        close(server);
        return;
    }
    set_nonblocking(server);
    std::printf("visualizer: http://localhost:%d\n", k_visualizer_port);

    std::vector<int> clients;
    auto next = std::chrono::steady_clock::now();
    int publish_divider = 0;

    while (running_.load())
    {
        for (;;)
        {
            const int client = accept(server, nullptr, nullptr);
            if (client < 0)
            {
                break;
            }

            set_nonblocking(client);
            char req[1024] = {};
            const ssize_t nread = recv(client, req, sizeof(req) - 1, 0);
            if (nread <= 0)
            {
                close(client);
                continue;
            }
            if (std::strncmp(req, "GET /events", 11) == 0)
            {
                const std::string header =
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/event-stream\r\n"
                    "Cache-Control: no-cache\r\n"
                    "Connection: keep-alive\r\n"
                    "Access-Control-Allow-Origin: *\r\n\r\n";
                if (send_text(client, header))
                {
                    clients.push_back(client);
                }
                else
                {
                    close(client);
                }
            }
            else
            {
                const std::string body = visualizer_html();
                const std::string header = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                                           "Cache-Control: no-cache\r\nContent-Length: " +
                                           std::to_string(body.size()) + "\r\n\r\n";
                send_text(client, header + body);
                close(client);
            }
        }

        control::msg_log_t log{};
        if (msg::read(sub_log_, log) == msg::status::ok && ++publish_divider >= 20)
        {
            publish_divider = 0;
            const std::string event = "data: " + log_to_json(log) + "\n\n";
            for (auto it = clients.begin(); it != clients.end();)
            {
                if (send_text(*it, event))
                {
                    ++it;
                }
                else
                {
                    close(*it);
                    it = clients.erase(it);
                }
            }
        }

        sleep_until_tick(next, 1000.0f);
    }

    for (int client : clients)
    {
        close(client);
    }
    close(server);
}

}  // namespace controller

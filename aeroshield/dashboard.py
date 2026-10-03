from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import json
import threading
import time

from simulator import make, SCENARIOS
from sensor_fusion import fuse
from trajectory import predict, motion
from threat_engine import decide

STATE = {
    "t": 0.0,
    "scenario": "HIGH RISK",
    "history": [],
    "data": None,
}

HTML = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>AEROSHIELD GCS</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#050d14;color:#e7f1f6;font:14px Arial,sans-serif}
header{padding:12px 18px;background:#091923;border-bottom:1px solid #294352;display:flex;justify-content:space-between;align-items:center;gap:15px;flex-wrap:wrap}
h1{margin:0;font-size:22px;letter-spacing:2px}
.sub{color:#829cab;font-size:11px;margin-top:3px}
button{background:#102d3d;color:#dcecf3;border:1px solid #345366;padding:7px 10px;margin:2px;border-radius:5px;cursor:pointer}
button:hover{background:#173f53}
.grid{display:grid;grid-template-columns:1.05fr 1.35fr 1fr;gap:10px;padding:10px}
.card{background:#091923;border:1px solid #263f4c;border-radius:8px;padding:11px;min-height:125px}
.title{color:#789aaa;font-size:10px;letter-spacing:1.5px;margin-bottom:8px}
.big{font-size:27px;font-weight:bold}
.row{display:flex;justify-content:space-between;gap:10px;margin:5px 0}
.bar{height:7px;background:#18303c;margin:3px 0 6px;border-radius:4px;overflow:hidden}
.fill{height:100%;background:#43c98d;transition:width .35s}
pre{white-space:pre-wrap;font-size:10px;color:#9db2bd;margin:0}
.alert{font-size:16px;font-weight:bold}
.radar-card{grid-row:span 2}
#radar{width:100%;height:420px;background:#030b11;border:1px solid #1d3945;border-radius:5px;display:block}
.legend{font-size:10px;color:#789aaa;margin-top:5px;display:flex;gap:14px;flex-wrap:wrap}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
.green{background:#43c98d}.red{background:#ff5d6c}.yellow{background:#e8c95d}.cyan{background:#56c7e8}
.status{padding:4px 7px;border:1px solid #31515f;border-radius:4px;font-size:10px}
@media(max-width:900px){.grid{grid-template-columns:1fr}.radar-card{grid-row:auto}}
</style>
</head>
<body>
<header>
<div><h1>AEROSHIELD</h1><div class="sub">MULTI-SENSOR COUNTER-UAV INTELLIGENCE • GROUND CONTROL CONSOLE</div></div>
<div>
<span class="status" id="live">● LIVE SIMULATION</span>
<button onclick="setScenario('NORMAL')">NORMAL</button>
<button onclick="setScenario('SUSPICIOUS')">SUSPICIOUS</button>
<button onclick="setScenario('HIGH RISK')">HIGH RISK</button>
<button onclick="setScenario('SENSOR DISAGREEMENT')">DISAGREEMENT</button>
</div>
</header>

<div class="grid">
<div class="card">
<div class="title">THREAT ASSESSMENT</div>
<div id="level" class="big">--</div>
<div class="row"><span>Threat Score</span><b id="score">--</b></div>
<div class="row"><span>Primary Track</span><b id="track">--</b></div>
<div class="row"><span>Evidence</span><b id="evidence">--</b></div>
<div class="row"><span>Motion</span><b id="motion">--</b></div>
</div>

<div class="card radar-card">
<div class="title">2D ACTIVE RADAR • REAL-TIME TRACKING</div>
<canvas id="radar"></canvas>
<div class="legend">
<span><i class="dot green"></i>corroborated track</span>
<span><i class="dot red"></i>priority track</span>
<span><i class="dot yellow"></i>secondary track</span>
<span><i class="dot cyan"></i>predicted path</span>
</div>
</div>

<div class="card">
<div class="title">DECISION ENGINE</div>
<div id="action" class="alert">--</div>
<div class="row"><span>Trajectory</span><b id="traj">--</b></div>
<div class="row"><span>Prediction Horizon</span><b id="pred">--</b></div>
<div class="row"><span>Sensor State</span><b id="sensorstate">--</b></div>
</div>

<div class="card">
<div class="title">RADAR TRACK DATA</div>
<div class="row"><span>Distance</span><b id="distance">--</b></div>
<div class="row"><span>Altitude</span><b id="altitude">--</b></div>
<div class="row"><span>Speed</span><b id="speed">--</b></div>
<div class="row"><span>Heading</span><b id="heading">--</b></div>
<div class="row"><span>Active Tracks</span><b id="tracks">--</b></div>
</div>

<div class="card">
<div class="title">SENSOR FUSION</div>
<div id="fusion"></div>
</div>

<div class="card">
<div class="title">GPS / RF / VISION</div>
<pre id="details"></pre>
</div>

<div class="card">
<div class="title">UAV TELEMETRY / ENVIRONMENT</div>
<pre id="env"></pre>
</div>

<div class="card">
<div class="title">TRAJECTORY PREDICTION</div>
<pre id="prediction"></pre>
</div>

<div class="card">
<div class="title">INTELLIGENCE PRINCIPLE</div>
<p style="color:#a7bac4;line-height:1.45;margin:0">
Independent sensor evidence is fused with movement behavior. Corroborating sensors increase confidence; disagreement is explicitly surfaced instead of being hidden.
</p>
</div>
</div>

<script>
let latest=null;
let sweep=0;
let animationId=null;

async function setScenario(name){
    await fetch('/scenario?name='+encodeURIComponent(name));
}

function $(id){return document.getElementById(id)}

function updateText(s){
    $("level").textContent=s.decision.level;
    $("score").textContent=Math.round(s.score)+"/100";
    $("track").textContent="UAV-"+s.radar.track_id;
    $("evidence").textContent=s.decision.evidence;
    $("motion").textContent=s.decision.motion;
    $("action").textContent=s.decision.action;
    $("traj").textContent=s.pred.direction;
    $("pred").textContent=s.pred.horizon+" sec";
    $("sensorstate").textContent=s.disagreement.length ? "DISAGREEMENT" : "CORROBORATED";
    $("distance").textContent=Math.round(s.radar.distance)+" m";
    $("altitude").textContent=Math.round(s.radar.altitude)+" m";
    $("speed").textContent=s.radar.speed.toFixed(1)+" m/s";
    $("heading").textContent=Math.round(s.radar.heading)+"°";
    $("tracks").textContent=s.tracks.length;

    let h="";
    for(const [k,v] of Object.entries(s.components)){
        h += '<div class="row"><span>'+k.toUpperCase()+'</span><b>'+Math.round(v)+'%</b></div>';
        h += '<div class="bar"><div class="fill" style="width:'+Math.max(0,Math.min(100,v))+'%"></div></div>';
    }
    $("fusion").innerHTML=h;
    $("details").textContent=JSON.stringify({
        GPS:s.gps, RF:s.rf, VISION:s.vision,
        SENSOR_DISAGREEMENT:s.disagreement
    },null,2);
    $("env").textContent=JSON.stringify({
        TELEMETRY:s.telemetry, ENVIRONMENT:s.environment
    },null,2);
    $("prediction").textContent=JSON.stringify(s.pred,null,2);
}

async function tick(){
    try{
        latest=await (await fetch('/state')).json();
        updateText(latest);
    }catch(e){}
}

function drawRadar(){
    const c=$("radar");
    const rect=c.getBoundingClientRect();
    const d=window.devicePixelRatio||1;
    const w=Math.max(300,rect.width), h=Math.max(300,rect.height);
    if(c.width!==Math.floor(w*d) || c.height!==Math.floor(h*d)){
        c.width=Math.floor(w*d); c.height=Math.floor(h*d);
    }
    const x=c.getContext("2d");
    x.setTransform(d,0,0,d,0,0);
    x.clearRect(0,0,w,h);

    const cx=w/2, cy=h/2;
    const R=Math.min(w,h)*0.39;

    // radar background
    x.fillStyle="#030b11"; x.fillRect(0,0,w,h);

    // range rings
    x.strokeStyle="#173742";
    x.lineWidth=1;
    for(let i=1;i<=4;i++){
        x.beginPath(); x.arc(cx,cy,R*i/4,0,Math.PI*2); x.stroke();
    }

    // cross hairs
    x.beginPath();
    x.moveTo(cx-R,cy); x.lineTo(cx+R,cy);
    x.moveTo(cx,cy-R); x.lineTo(cx,cy+R);
    x.stroke();

    // diagonals
    x.beginPath();
    x.moveTo(cx-R*.72,cy-R*.72); x.lineTo(cx+R*.72,cy+R*.72);
    x.moveTo(cx-R*.72,cy+R*.72); x.lineTo(cx+R*.72,cy-R*.72);
    x.strokeStyle="#102b35"; x.stroke();

    // labels
    x.fillStyle="#4f7380"; x.font="10px Arial";
    x.fillText("N",cx-4,cy-R-7);
    x.fillText("E",cx+R+7,cy+3);
    x.fillText("S",cx-4,cy+R+15);
    x.fillText("W",cx-R-16,cy+3);

    if(!latest){
        sweep=(sweep+0.025)%(Math.PI*2);
        animationId=requestAnimationFrame(drawRadar);
        return;
    }

    // rotating sweep
    sweep=(sweep+0.035)%(Math.PI*2);
    const sweepWidth=0.30;
    const grad=x.createRadialGradient(cx,cy,0,cx,cy,R);
    grad.addColorStop(0,"rgba(67,201,141,.30)");
    grad.addColorStop(1,"rgba(67,201,141,0)");
    x.fillStyle=grad;
    x.beginPath();
    x.moveTo(cx,cy);
    x.arc(cx,cy,R,sweep-sweepWidth,sweep);
    x.closePath(); x.fill();

    x.strokeStyle="rgba(67,201,141,.85)";
    x.lineWidth=2;
    x.beginPath();
    x.moveTo(cx,cy);
    x.lineTo(cx+R*Math.cos(sweep),cy+R*Math.sin(sweep));
    x.stroke();

    // predicted path for primary target
    const p=latest.radar;
    const heading=(p.heading-90)*Math.PI/180;
    const px=cx+Math.min(R*.75,R*p.distance/600)*Math.cos(heading);
    const py=cy+Math.min(R*.75,R*p.distance/600)*Math.sin(heading);
    const pathLen=Math.min(R*.34, p.speed*5);
    x.strokeStyle="#56c7e8";
    x.setLineDash([6,5]);
    x.beginPath();
    x.moveTo(px,py);
    x.lineTo(px+pathLen*Math.cos(heading),py+pathLen*Math.sin(heading));
    x.stroke();
    x.setLineDash([]);

    // tracks
    latest.tracks.forEach((p,i)=>{
        const a=(p.angle-90)*Math.PI/180;
        const rr=Math.min(R*.88,Math.max(8,p.distance/600*R));
        const tx=cx+rr*Math.cos(a);
        const ty=cy+rr*Math.sin(a);
        const primary=i===0;
        const high=latest.decision.level==="HIGH RISK" || latest.decision.level==="CRITICAL";
        const color=primary && high ? "#ff5d6c" : primary ? "#43c98d" : "#e8c95d";

        // history-like trail
        x.strokeStyle=primary ? "rgba(67,201,141,.25)" : "rgba(232,201,93,.18)";
        x.lineWidth=1;
        x.beginPath();
        x.moveTo(tx,ty);
        x.lineTo(cx+rr*Math.cos(a-0.13),cy+rr*Math.sin(a-0.13));
        x.lineTo(cx+rr*Math.cos(a-0.25),cy+rr*Math.sin(a-0.25));
        x.stroke();

        // target pulse
        const pulse=primary ? 7+3*Math.sin(Date.now()/180) : 5;
        x.fillStyle=color;
        x.beginPath(); x.arc(tx,ty,pulse,0,Math.PI*2); x.fill();
        x.fillStyle="#dcecf2"; x.font="10px Arial";
        x.fillText("UAV-"+p.track_id,tx+10,ty-7);

        if(primary){
            x.strokeStyle=color;
            x.beginPath(); x.arc(tx,ty,pulse+6,0,Math.PI*2); x.stroke();
        }
    });

    // center platform
    x.fillStyle="#56c7e8";
    x.beginPath(); x.arc(cx,cy,3,0,Math.PI*2); x.fill();

    animationId=requestAnimationFrame(drawRadar);
}

setInterval(tick,500);
tick();
drawRadar();
</script>
</body>
</html>"""

def compute():
    frame = make(STATE["t"], STATE["scenario"])
    score, components, confirmed, issues = fuse(
        frame["radar"], frame["gps"], frame["rf"], frame["vision"]
    )
    predicted = predict(frame["radar"], frame["gps"])
    movement = motion(STATE["history"], frame["radar"])
    decision = decide(score, confirmed, movement, issues)

    STATE["history"] = (
        STATE["history"] +
        [{"distance": frame["radar"]["distance"], "speed": frame["radar"]["speed"]}]
    )[-10:]

    STATE["data"] = {
        **frame,
        "score": score,
        "components": components,
        "confirmed": confirmed,
        "disagreement": issues,
        "pred": predicted,
        "decision": decision,
    }

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed=urlparse(self.path)

        if parsed.path == "/state":
            body=json.dumps(STATE["data"]).encode()
            self.send_response(200)
            self.send_header("Content-Type","application/json")
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == "/scenario":
            name=parse_qs(parsed.query).get("name",["HIGH RISK"])[0]
            if name in SCENARIOS:
                STATE["scenario"]=name
                STATE["history"]=[]
                STATE["t"]=0.0
            self.send_response(204)
            self.end_headers()
            return

        body=HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self,*args):
        pass

def main():
    compute()

    def loop():
        while True:
            STATE["t"] += 0.5
            compute()
            time.sleep(0.5)

    threading.Thread(target=loop,daemon=True).start()
    print("AEROSHIELD FINAL: http://127.0.0.1:8080")
    HTTPServer(("0.0.0.0",8080),Handler).serve_forever()

if __name__=="__main__":
    main()

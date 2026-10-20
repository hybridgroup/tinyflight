import * as THREE from 'three';
import {OrbitControls} from 'three/addons/OrbitControls.js';

// The sim works in NED. three.js is Y up, so north is -z, east is +x and down is -y.
const ned = (x, y, z) => new THREE.Vector3(y, -z, -x);

// Motors in mixer order, from QuadX in pkg/mixer/mixer.go. FR and RL spin counter clockwise.
const ARM = 0.023;
const MOTORS = [
  {x: ARM, y: -ARM, ccw: false},
  {x: ARM, y: ARM, ccw: true},
  {x: -ARM, y: -ARM, ccw: true},
  {x: -ARM, y: ARM, ccw: false},
];

export class View {
  constructor(canvas) {
    this.renderer = new THREE.WebGLRenderer({canvas, antialias: true});
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.shadowMap.enabled = true;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xbcd3e6);
    this.scene.fog = new THREE.Fog(0xbcd3e6, 6, 20);
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.01, 100);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.mode = 'chase';
    this.chaseYaw = 0;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x6b5b45, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2);
    sun.position.set(2, 6, 3);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, {left: -3, right: 3, top: 3, bottom: -3, near: 0.5, far: 15});
    this.sun = sun;
    this.scene.add(sun, sun.target);

    this.scene.add(floor());
    this.drone = drone();
    this.scene.add(this.drone.root);

    // Where the firmware thinks it is.
    this.ghost = new THREE.Mesh(
      new THREE.TorusGeometry(0.035, 0.0015, 8, 32),
      new THREE.MeshBasicMaterial({color: 0xff7a00, transparent: true, opacity: 0.7}));
    this.ghost.rotation.x = Math.PI / 2;
    this.scene.add(this.ghost);

    // A line down to the floor helps judge height.
    this.drop = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({color: 0x333333, dashSize: 0.02, gapSize: 0.02}));
    this.scene.add(this.drop);

    this.setCamera('chase');
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
  }

  resize() {
    const c = this.renderer.domElement;
    this.renderer.setSize(c.clientWidth, c.clientHeight, false);
    this.camera.aspect = c.clientWidth / Math.max(c.clientHeight, 1);
    this.camera.updateProjectionMatrix();
  }

  // chase follows behind, pilot stands still like flying line of sight, orbit is free.
  setCamera(mode) {
    this.mode = mode;
    this.controls.enabled = mode === 'orbit';
    if (mode === 'pilot') {
      this.camera.position.copy(ned(-1.5, 0, -1.6));
    }
    if (mode === 'orbit') {
      this.camera.position.copy(this.drone.root.position).add(new THREE.Vector3(0.6, 0.5, 0.8));
      this.controls.target.copy(this.drone.root.position);
    }
  }

  // update draws the state record from boards/sim/world.go, with offsets in s.
  update(state, s, dt) {
    const d = this.drone;
    const pos = ned(state[s.pos], state[s.pos + 1], state[s.pos + 2]);
    d.root.position.copy(pos);
    const q = state.subarray(s.quat, s.quat + 4);
    d.root.quaternion.set(q[2], -q[3], -q[1], q[0]);

    for (let i = 0; i < 4; i++) {
      const cmd = state[s.motors + i];
      const spin = (MOTORS[i].ccw ? 1 : -1) * cmd * 120 * dt;
      d.props[i].rotation.y += spin;
      d.blurs[i].material.opacity = Math.min(cmd * 1.5, 0.5);
      d.blades[i].visible = cmd < 0.3;
    }
    const led = new THREE.Color(state[s.led] / 32, state[s.led + 1] / 32, state[s.led + 2] / 32);
    d.led.material.color.copy(led);
    d.led.material.emissive.copy(led);

    this.ghost.position.copy(ned(state[s.est], state[s.est + 1], state[s.est + 2]));
    this.ghost.position.y = Math.max(this.ghost.position.y, 0.002);

    const p = this.drop.geometry.attributes.position;
    p.setXYZ(0, pos.x, pos.y, pos.z);
    p.setXYZ(1, pos.x, 0, pos.z);
    p.needsUpdate = true;
    this.drop.computeLineDistances();

    this.sun.position.set(pos.x + 2, 6, pos.z + 3);
    this.sun.target.position.copy(pos);
    this.follow(pos, dt);
    this.renderer.render(this.scene, this.camera);
  }

  follow(pos, dt) {
    if (this.mode === 'orbit') {
      this.controls.target.lerp(pos, Math.min(dt * 5, 1));
      this.controls.update();
      return;
    }
    if (this.mode === 'chase') {
      const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(this.drone.root.quaternion);
      const yaw = Math.atan2(-fwd.x, -fwd.z);
      let dy = yaw - this.chaseYaw;
      dy = Math.atan2(Math.sin(dy), Math.cos(dy));
      this.chaseYaw += dy * Math.min(dt * 2, 1);
      const back = new THREE.Vector3(Math.sin(this.chaseYaw), 0, Math.cos(this.chaseYaw)).multiplyScalar(0.45);
      const want = pos.clone().add(back).add(new THREE.Vector3(0, 0.18, 0));
      want.y = Math.max(want.y, 0.08);
      this.camera.position.copy(want);
    }
    this.camera.lookAt(pos);
  }
}

// floor has a fine pattern, since optical flow needs texture and so does the pilot judging drift.
function floor() {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d');
  for (let y = 0; y < 8; y++) {
    for (let x = 0; x < 8; x++) {
      g.fillStyle = (x + y) % 2 ? '#c9b48f' : '#b39c76';
      g.fillRect(x * 64, y * 64, 64, 64);
    }
  }
  for (let i = 0; i < 1500; i++) {
    g.fillStyle = `rgba(60,40,20,${Math.random() * 0.25})`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 3, 3);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  // One canvas covers 0.8 m, so each tile is 10 cm.
  const size = 40;
  tex.repeat.set(size / 0.8, size / 0.8);
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshStandardMaterial({map: tex}));
  mesh.rotation.x = -Math.PI / 2;
  mesh.receiveShadow = true;

  const group = new THREE.Group();
  group.add(mesh);
  const grid = new THREE.GridHelper(size, size, 0x5a4a32, 0x7a6a50);
  grid.position.y = 0.001;
  grid.material.transparent = true;
  grid.material.opacity = 0.5;
  group.add(grid);
  const north = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(-0.4, 0.002, 0), 0.3, 0xcc2222, 0.06, 0.04);
  group.add(north);
  return group;
}

// drone is a StampFly sized model, about 8 cm across. Its local axes are body FRD mapped like ned().
function drone() {
  const root = new THREE.Group();
  const body = new THREE.MeshStandardMaterial({color: 0x2b2f36, roughness: 0.6});
  const accent = new THREE.MeshStandardMaterial({color: 0xe8e8e8, roughness: 0.5});
  const shadow = (m) => { m.castShadow = true; return m; };

  root.add(shadow(new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.012, 0.042), body)));
  const nose = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.006, 0.01), accent));
  nose.position.set(0, 0.006, -0.022);
  root.add(nose);

  const props = [], blurs = [], blades = [];
  for (const m of MOTORS) {
    const at = ned(m.x, m.y, 0);
    const arm = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.004, at.length()), body));
    arm.position.copy(at.clone().multiplyScalar(0.5));
    arm.lookAt(at);
    root.add(arm);

    const guard = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.02, 0.0015, 6, 24), accent));
    guard.rotation.x = Math.PI / 2;
    guard.position.copy(at).setY(0.004);
    root.add(guard);

    const motor = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.008, 12), body));
    motor.position.copy(at).setY(0.002);
    root.add(motor);

    const prop = new THREE.Group();
    prop.position.copy(at).setY(0.007);
    const blade = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.001, 0.005),
      new THREE.MeshStandardMaterial({color: m.ccw ? 0x1f6fd1 : 0xd13b1f})));
    prop.add(blade);
    const blur = new THREE.Mesh(new THREE.CircleGeometry(0.018, 24),
      new THREE.MeshBasicMaterial({color: m.ccw ? 0x1f6fd1 : 0xd13b1f, transparent: true, opacity: 0, depthWrite: false}));
    blur.rotation.x = -Math.PI / 2;
    prop.add(blur);
    root.add(prop);
    props.push(prop);
    blurs.push(blur);
    blades.push(blade);
  }

  const led = new THREE.Mesh(new THREE.SphereGeometry(0.004, 12, 8),
    new THREE.MeshStandardMaterial({color: 0x000000, emissive: 0x000000, emissiveIntensity: 3}));
  led.position.set(0, 0.008, 0.012);
  root.add(led);
  return {root, props, blurs, blades, led};
}

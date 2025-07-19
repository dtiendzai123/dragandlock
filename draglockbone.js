const GamePackages = {
  GamePackage1: "com.dts.freefireth",
  GamePackage2: "com.dts.freefiremax"
};
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }
  normalize() {
    const len = this.length();
    return len > 0 ? this.multiplyScalar(1 / len) : new Vector3();
  }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q;
    this.A = 1; this.B = 0; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }

  filter(z) {
    if (isNaN(this.x)) {
      this.x = z;
      this.cov = 1;
    } else {
      const predX = this.A * this.x;
      const predCov = this.A * this.cov * this.A + this.Q;
      const K = predCov * this.C / (this.C * predCov * this.C + this.R);
      this.x = predX + K * (z - this.C * predX);
      this.cov = predCov - K * this.C * predCov;
    }
    return this.x;
  }
}

function quaternionToMatrix(q) {
  const { x, y, z, w } = q;
  return [
    1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w,
    2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
    2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y
  ];
}

function transformWithBindpose(pos, rot, scale, bindpose) {
  const mat = quaternionToMatrix(rot);
  const sx = scale.x, sy = scale.y, sz = scale.z;
  const model = [
    mat[0]*sx, mat[1]*sy, mat[2]*sz, pos.x,
    mat[3]*sx, mat[4]*sy, mat[5]*sz, pos.y,
    mat[6]*sx, mat[7]*sy, mat[8]*sz, pos.z
  ];
  const b = bindpose;
  return new Vector3(
    b.e00 * model[3] + b.e01 * model[7] + b.e02 * model[11] + b.e03,
    b.e10 * model[3] + b.e11 * model[7] + b.e12 * model[11] + b.e13,
    b.e20 * model[3] + b.e21 * model[7] + b.e22 * model[11] + b.e23
  );
}

class AutoLockAndDragHeadSystem {
  constructor() {
    this.kalmanX = new KalmanFilter();
    this.kalmanY = new KalmanFilter();
    this.kalmanZ = new KalmanFilter();
    this.lastLockTime = 0;
  }

  getSmoothedPosition(pos) {
    return new Vector3(
      this.kalmanX.filter(pos.x),
      this.kalmanY.filter(pos.y),
      this.kalmanZ.filter(pos.z)
    );
  }

  isCrosshairRed() {
    // Placeholder, thay bằng API thực tế nếu có
    return typeof GameAPI !== 'undefined' && GameAPI.isCrosshairRed?.() === true;
  }

  setAim(x, y, z) {
    // Gọi API game thực sự ở đây
    if (typeof GameAPI !== 'undefined' && GameAPI.setAimTarget) {
      GameAPI.setAimTarget(x, y, z);
    } else {
      console.log("🎯 AutoLock →", x.toFixed(4), y.toFixed(4), z.toFixed(4));
    }
  }

  lockToHead() {
    // Thông tin bone_Head
    const position = { x: -0.0456970781, y: -0.004478302, z: -0.0200432576 };
    const rotation = { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 };
    const scale = { x: 0.99999994, y: 1.00000012, z: 1.0 };
    const bindpose = {
      e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
      e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
      e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
      e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
    };

    const worldHead = transformWithBindpose(position, rotation, scale, bindpose);
    const smoothHead = this.getSmoothedPosition(worldHead);
    this.setAim(smoothHead.x, smoothHead.y, smoothHead.z);
  }

  runLoop() {
    const loop = () => {
      if (this.isCrosshairRed()) {
        this.lockToHead();
        this.lastLockTime = Date.now();
      }
      setTimeout(loop, 16); // ~60fps
    };
    loop();
  }
}

// Chạy hệ thống
const aimBot = new AutoLockAndDragHeadSystem();
aimBot.runLoop();

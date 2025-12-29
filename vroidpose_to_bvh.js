// vroidpose_to_bvh.js
// Usage: node vroidpose_to_bvh.js input.vroidpose out.bvh

const fs = require('fs');

// VRoid to BVH bone mapping
const BONE_MAP = {
  "Hips": "hips",
  "Spine": "spine",
  "Chest": "chest",
  "UpperChest": "upperChest",
  "Neck": "neck",
  "Head": "head",

  "LeftShoulder": "leftShoulder",
  "LeftUpperArm": "leftUpperArm",
  "LeftLowerArm": "leftLowerArm",
  "LeftHand": "leftHand",

  "RightShoulder": "rightShoulder",
  "RightUpperArm": "rightUpperArm",
  "RightLowerArm": "rightLowerArm",
  "RightHand": "rightHand",

  "LeftUpperLeg": "leftUpperLeg",
  "LeftLowerLeg": "leftLowerLeg",
  "LeftFoot": "leftFoot",
  "LeftToes": "leftToes",

  "RightUpperLeg": "rightUpperLeg",
  "RightLowerLeg": "rightLowerLeg",
  "RightFoot": "rightFoot",
  "RightToes": "rightToes",
};
// Quaternion → Euler(deg) (XYZ)
function quatToEulerXYZDeg(qx, qy, qz, qw) {
  const ysqr = qy * qy;

  const t0 = +2.0 * (qw * qx + qy * qz);
  const t1 = +1.0 - 2.0 * (qx * qx + ysqr);
  const rollX = Math.atan2(t0, t1);

  let t2 = +2.0 * (qw * qy - qz * qx);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  const pitchY = Math.asin(t2);

  const t3 = +2.0 * (qw * qz + qx * qy);
  const t4 = +1.0 - 2.0 * (ysqr + qz * qz);
  const yawZ = Math.atan2(t3, t4);

  const rad2deg = (r) => r * (180 / Math.PI);
  return { x: rad2deg(rollX), y: rad2deg(pitchY), z: rad2deg(yawZ) };
}

// load .vroidpose
function loadVroidPose(p) {
  const raw = fs.readFileSync(p, 'utf8');
  const json = JSON.parse(raw);

  const bones = json.BoneDefinition;
  if (!bones) throw new Error('Invalid .vroidpose: no BoneDefinition found');

  return { bones };
}
function buildBVHOneFrame(pose) {
  const lines = [];
  lines.push("HIERARCHY");

  const hierarchy = [
    { name: "hips", parent: null, offset: [0, 0, 0] },
    { name: "spine", parent: "hips", offset: [0, 10, 0] },
    { name: "chest", parent: "spine", offset: [0, 10, 0] },
    { name: "upperChest", parent: "chest", offset: [0, 10, 0] },
    { name: "neck", parent: "upperChest", offset: [0, 10, 0] },
    { name: "head", parent: "neck", offset: [0, 10, 0] },

    { name: "leftShoulder", parent: "upperChest", offset: [2, 8, 0] },
    { name: "leftUpperArm", parent: "leftShoulder", offset: [6, 0, 0] },
    { name: "leftLowerArm", parent: "leftUpperArm", offset: [10, 0, 0] },
    { name: "leftHand", parent: "leftLowerArm", offset: [8, 0, 0] },

    { name: "rightShoulder", parent: "upperChest", offset: [-2, 8, 0] },
    { name: "rightUpperArm", parent: "rightShoulder", offset: [-6, 0, 0] },
    { name: "rightLowerArm", parent: "rightUpperArm", offset: [-10, 0, 0] },
    { name: "rightHand", parent: "rightLowerArm", offset: [-8, 0, 0] },

    { name: "leftUpperLeg", parent: "hips", offset: [4, -10, 0] },
    { name: "leftLowerLeg", parent: "leftUpperLeg", offset: [0, -20, 0] },
    { name: "leftFoot", parent: "leftLowerLeg", offset: [0, -8, 4] },
    { name: "leftToes", parent: "leftFoot", offset: [0, 0, 4] },

    { name: "rightUpperLeg", parent: "hips", offset: [-4, -10, 0] },
    { name: "rightLowerLeg", parent: "rightUpperLeg", offset: [0, -20, 0] },
    { name: "rightFoot", parent: "rightLowerLeg", offset: [0, -8, 4] },
    { name: "rightToes", parent: "rightFoot", offset: [0, 0, 4] },
  ];

  const childrenOf = (parent) => hierarchy.filter(b => b.parent === parent);

  function writeJoint(bone, depth) {
    const indent = "  ".repeat(depth);
    lines.push(`${indent}JOINT ${bone.name}`);
    lines.push(`${indent}{`);
    lines.push(`${indent}  OFFSET ${bone.offset[0]} ${bone.offset[1]} ${bone.offset[2]}`);
    lines.push(`${indent}  CHANNELS 3 Zrotation Xrotation Yrotation`);
    for (const child of childrenOf(bone.name)) {
      writeJoint(child, depth + 1);
    }
    lines.push(`${indent}}`);
  }

  // Root hips
  const root = hierarchy.find(b => b.parent === null);
  lines.push(`ROOT ${root.name}`);
  lines.push("{");
  lines.push(`  OFFSET ${root.offset[0]} ${root.offset[1]} ${root.offset[2]}`);
  lines.push(`  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation`);
  for (const child of childrenOf(root.name)) {
    writeJoint(child, 1);
  }
  lines.push("}");

  // Motion
  lines.push("MOTION");
  lines.push("Frames: 1");
  lines.push("Frame Time: 0.0333333");

  const frameValues = [];
  const bones = pose.bones;

  // hips position
  const hipsPos = bones.HipsPosition || {x:0,y:0,z:0};
  frameValues.push(hipsPos.x, hipsPos.y, hipsPos.z);

  //bone rotate 
  for (const [boneName, vrmBone] of Object.entries(BONE_MAP)) {
    const rot = bones[boneName];
    if (!rot || rot.w === undefined) {
      frameValues.push(0,0,0);
      continue;
    }
    const euler = quatToEulerXYZDeg(rot.x, rot.y, rot.z, rot.w);

      euler.y = -euler.y;
      euler.z = -euler.z;
/*
    if (boneName === "Head") {
      euler.y = -euler.y;
    }
    if (boneName === "Neck") {
      euler.y = -euler.y;
    }
*/

    frameValues.push(euler.z.toFixed(3), euler.x.toFixed(3), euler.y.toFixed(3));
    //frameValues.push(euler.z.toFixed(3), euler.x.toFixed(3), euler.y.toFixed(3));
    //frameValues.push(euler.z, euler.x, euler.y);
  }

  lines.push(frameValues.join(" "));
  return lines.join("\n");
}

function main() {
  if (process.argv.length < 4) {
    console.error("Usage: node vroidpose_to_bvh.js input.vroidpose out.bvh");
    process.exit(1);
  }

  const inputFile = process.argv[2];
  const outBvh = process.argv[3];

  const pose = loadVroidPose(inputFile);
  const bvh = buildBVHOneFrame(pose);
  fs.writeFileSync(outBvh, bvh);

  console.log(`Exported BVH -> ${outBvh}`);
}

main();


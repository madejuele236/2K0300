import fs from "node:fs";

const lines = fs.readFileSync(0, "utf8").trim().split(/\r?\n/).filter(Boolean);
if (lines.length !== 3) {
  throw new Error(`expected 3 telemetry frames, got ${lines.length}`);
}

const frames = lines.map((line) => JSON.parse(line));
for (const [index, frame] of frames.entries()) {
  if (frame.type !== "telemetry") {
    throw new Error(`frame ${index} is not telemetry`);
  }
  for (const key of ["effective_speed_target", "left_speed_target", "right_speed_target"]) {
    if (typeof frame[key] !== "number" || !Number.isFinite(frame[key])) {
      throw new Error(`frame ${index} ${key} is not a finite JSON number`);
    }
  }
}

if (!(frames[0].applied_turn_output > 0 && frames[1].applied_turn_output < 0)) {
  throw new Error("positive and negative production turn paths were not both emitted");
}
if (frames[0].left_speed_target !== frames[1].right_speed_target ||
    frames[0].right_speed_target !== frames[1].left_speed_target) {
  throw new Error("positive/negative turn telemetry is not mirrored");
}
if (frames[2].applied_turn_output !== 0 ||
    frames[2].left_speed_target !== 5000 ||
    frames[2].right_speed_target !== 5000) {
  throw new Error("zero reachable turn boundary telemetry mismatch");
}

console.log("wheel_target_parameter_telemetry_json_check passed");

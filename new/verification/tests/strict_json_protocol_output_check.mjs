import fs from "node:fs";

function fail(message) {
  throw new Error(message);
}

function readJson(path) {
  let parsed;
  const text = fs.readFileSync(path, "utf8");
  try {
    parsed = JSON.parse(text);
  } catch (error) {
    fail(`${path}: browser-compatible JSON.parse rejected output: ${error.message}`);
  }
  if (parsed === null || typeof parsed !== "object" || Array.isArray(parsed)) {
    fail(`${path}: JSON root must be an object`);
  }
  assertFiniteNumbers(parsed, "$", path);
  return parsed;
}

function assertFiniteNumbers(value, jsonPath, sourcePath) {
  if (typeof value === "number") {
    if (!Number.isFinite(value)) {
      fail(`${sourcePath}: ${jsonPath} is not a finite browser number`);
    }
    return;
  }
  if (Array.isArray(value)) {
    value.forEach((item, index) => assertFiniteNumbers(item, `${jsonPath}[${index}]`, sourcePath));
    return;
  }
  if (value !== null && typeof value === "object") {
    for (const [key, item] of Object.entries(value)) {
      assertFiniteNumbers(item, `${jsonPath}.${key}`, sourcePath);
    }
  }
}

function get(root, dottedPath) {
  let value = root;
  for (const key of dottedPath.split(".")) {
    if (value === null || typeof value !== "object" || !(key in value)) {
      fail(`${dottedPath}: required field is missing`);
    }
    value = value[key];
  }
  return value;
}

function expectValue(root, dottedPath, expected) {
  const actual = get(root, dottedPath);
  if (typeof actual !== typeof expected || !Object.is(actual, expected)) {
    fail(`${dottedPath}: expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`);
  }
}

function expectObject(root, dottedPath) {
  const actual = get(root, dottedPath);
  if (actual === null || typeof actual !== "object" || Array.isArray(actual)) {
    fail(`${dottedPath}: expected object`);
  }
}

function checkActuatorFacts(root, prefix, expected) {
  const field = (name) => (prefix.length === 0 ? name : `${prefix}.${name}`);
  expectValue(root, field("left_drive_pwm_command"), expected.leftCommand);
  expectValue(root, field("right_drive_pwm_command"), expected.rightCommand);
  expectValue(root, field("left_drive_pwm_unconstrained"), expected.leftUnconstrained);
  expectValue(root, field("right_drive_pwm_unconstrained"), expected.rightUnconstrained);
  expectValue(root, field("left_drive_pwm_requested"), expected.leftRequested);
  expectValue(root, field("right_drive_pwm_requested"), expected.rightRequested);
  expectValue(root, field("left_drive_pwm_desired"), expected.leftDesired);
  expectValue(root, field("right_drive_pwm_desired"), expected.rightDesired);
  expectValue(root, field("left_drive_pwm_step_limited"), expected.leftStepLimited);
  expectValue(root, field("right_drive_pwm_step_limited"), expected.rightStepLimited);
  expectValue(root, field("left_drive_pwm_reverse_suppressed"), expected.leftReverseSuppressed);
  expectValue(root, field("right_drive_pwm_reverse_suppressed"), expected.rightReverseSuppressed);
  expectValue(root, field("left_drive_pwm_floor_adjusted"), expected.leftFloorAdjusted);
  expectValue(root, field("right_drive_pwm_floor_adjusted"), expected.rightFloorAdjusted);
  expectValue(root, field("left_pid_error"), expected.leftPidError);
  expectValue(root, field("right_pid_error"), expected.rightPidError);
  expectValue(root, field("left_pid_integral"), expected.leftPidIntegral);
  expectValue(root, field("right_pid_integral"), expected.rightPidIntegral);
  expectValue(root, field("left_pid_integral_candidate"), expected.leftPidIntegralCandidate);
  expectValue(root, field("right_pid_integral_candidate"), expected.rightPidIntegralCandidate);
  expectValue(root, field("left_pid_anti_windup_active"), expected.leftAntiWindupActive);
  expectValue(root, field("right_pid_anti_windup_active"), expected.rightAntiWindupActive);
  expectValue(root, field("left_pid_anti_windup_reason"), expected.leftAntiWindupReason);
  expectValue(root, field("right_pid_anti_windup_reason"), expected.rightAntiWindupReason);
  expectValue(root, field("actuators_armed"), expected.actuatorsArmed);
  expectValue(root, field("last_confirmed_left_drive_pwm"), expected.lastConfirmedLeft);
  expectValue(root, field("last_confirmed_right_drive_pwm"), expected.lastConfirmedRight);
  expectValue(root, field("last_confirmed_left_brushless_pwm"), expected.lastConfirmedLeftBrushless);
  expectValue(root, field("last_confirmed_right_brushless_pwm"), expected.lastConfirmedRightBrushless);
}

function checkAssistant(path) {
  const root = readJson(path);
  expectValue(root, "type", "telemetry");
  expectValue(root, "motion_phase", "RUNNING");
  checkActuatorFacts(root, "", {
    leftCommand: 120,
    rightCommand: 130,
    leftUnconstrained: 145.5,
    rightUnconstrained: -50.25,
    leftRequested: 146,
    rightRequested: -50,
    leftDesired: 146,
    rightDesired: 0,
    leftStepLimited: true,
    rightStepLimited: false,
    leftReverseSuppressed: false,
    rightReverseSuppressed: true,
    leftFloorAdjusted: false,
    rightFloorAdjusted: false,
    leftPidError: 2.5,
    rightPidError: -1.5,
    leftPidIntegral: 9,
    rightPidIntegral: 4,
    leftPidIntegralCandidate: 11.5,
    rightPidIntegralCandidate: 2.5,
    leftAntiWindupActive: true,
    rightAntiWindupActive: false,
    leftAntiWindupReason: "actuator_limit",
    rightAntiWindupReason: "none",
    actuatorsArmed: true,
    lastConfirmedLeft: 120,
    lastConfirmedRight: 130,
    lastConfirmedLeftBrushless: 500,
    lastConfirmedRightBrushless: 600,
  });
}

function checkAssistantYawFault(firstPath, laterPath) {
  const first = readJson(firstPath);
  const later = readJson(laterPath);
  for (const [label, root] of [["first", first], ["later", later]]) {
    expectValue(root, "type", "telemetry");
    expectValue(root, "motion_phase", "FAIL_SAFE_LATCHED");
    expectValue(root, "yaw_control.valid", false);
    expectValue(root, "yaw_control.reason", "nonfinite_arithmetic");
    if (label === "later") {
      expectValue(root, "safety_gate.veto_active", false);
      expectValue(root, "safety_gate.reason", "none");
    }
    expectValue(root, "actuators_armed", false);
    expectValue(root, "last_confirmed_left_drive_pwm", 0);
    expectValue(root, "last_confirmed_right_drive_pwm", 0);
  }
}

function checkSteeringMedia(configPath, imagePath) {
  const config = readJson(configPath);
  expectValue(config, "type", "config_snapshot");
  expectObject(config, "param_snapshot");
  expectValue(config, "param_snapshot.pwm_limit", 4321);
  expectValue(config, "param_snapshot.pwm_floor", 237);
  expectValue(config, "param_snapshot.prohibit_reverse_pwm", false);
  expectValue(config, "param_snapshot.drive_pwm_step_limit", 654);
  const expectedPid = {
    left_wheel_pid: {p: 81.5, i: 2.75, d: 0.625, integral_limit: 321, measurement_filter_alpha: 0.35},
    right_wheel_pid: {p: 97.5, i: 1.875, d: 0.125, integral_limit: 456, measurement_filter_alpha: 0.65},
  };
  for (const [side, fields] of Object.entries(expectedPid)) {
    expectObject(config, `param_snapshot.${side}`);
    for (const [name, value] of Object.entries(fields)) {
      expectValue(config, `param_snapshot.${side}.${name}`, value);
    }
  }

  const image = readJson(imagePath);
  expectValue(image, "type", "image_frame");
  expectObject(image, "steering_snapshot");
  expectObject(image, "steering_snapshot.actuator");
  checkActuatorFacts(image, "steering_snapshot.actuator", {
    leftCommand: 101,
    rightCommand: 102,
    leftUnconstrained: 125.5,
    rightUnconstrained: -25.5,
    leftRequested: 126,
    rightRequested: -26,
    leftDesired: 126,
    rightDesired: 0,
    leftStepLimited: true,
    rightStepLimited: false,
    leftReverseSuppressed: false,
    rightReverseSuppressed: true,
    leftFloorAdjusted: false,
    rightFloorAdjusted: false,
    leftPidError: 3,
    rightPidError: 0,
    leftPidIntegral: 7,
    rightPidIntegral: 0,
    leftPidIntegralCandidate: 10,
    rightPidIntegralCandidate: 0,
    leftAntiWindupActive: true,
    rightAntiWindupActive: false,
    leftAntiWindupReason: "actuator_limit",
    rightAntiWindupReason: "none",
    actuatorsArmed: true,
    lastConfirmedLeft: 101,
    lastConfirmedRight: 102,
    lastConfirmedLeftBrushless: 501,
    lastConfirmedRightBrushless: 502,
  });
}

const [mode, ...paths] = process.argv.slice(2);
if (mode === "assistant" && paths.length === 1) {
  checkAssistant(paths[0]);
} else if (mode === "assistant-yaw-fault" && paths.length === 2) {
  checkAssistantYawFault(paths[0], paths[1]);
} else if (mode === "steering-media" && paths.length === 2) {
  checkSteeringMedia(paths[0], paths[1]);
} else {
  fail("usage: strict_json_protocol_output_check.mjs assistant <json> | assistant-yaw-fault <first-json> <later-json> | steering-media <config-json> <image-header-json>");
}

console.log(`${mode} strict JSON.parse validation passed`);

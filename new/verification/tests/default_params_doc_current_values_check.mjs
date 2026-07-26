#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const configDir = path.resolve(scriptDir, "../../config");
const codeDir = path.resolve(scriptDir, "../../code");
const markdownPath = path.join(configDir, "default_params.md");
const jsonPath = path.join(configDir, "default_params.json");

function readRequired(filePath, purpose) {
  try {
    return fs.readFileSync(filePath, "utf8");
  } catch (error) {
    console.error(`default_params documentation check cannot read ${purpose}: ${filePath}`);
    console.error(error instanceof Error ? error.message : String(error));
    process.exit(2);
  }
}

const markdown = readRequired(markdownPath, "documentation");
let parameters;
try {
  parameters = JSON.parse(readRequired(jsonPath, "runtime parameter JSON"));
} catch (error) {
  console.error(`default_params documentation check cannot parse JSON: ${jsonPath}`);
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(2);
}
const paramStoreSource = readRequired(
  path.join(codeDir, "platform/param_store.cpp"),
  "parameter loader source",
);
const referenceValidationSource = readRequired(
  path.join(codeDir, "port/runtime_parameter_validation.hpp"),
  "reference alignment validation source",
);
const runtimeParameterTypesSource = readRequired(
  path.join(codeDir, "port/runtime_parameter_types.hpp"),
  "runtime parameter bound declarations",
);
const assistantProtocolSource = readRequired(
  path.join(codeDir, "transport/assistant_protocol.cpp"),
  "assistant telemetry protocol source",
);
const mediaProtocolSource = readRequired(
  path.join(codeDir, "transport/steering_media_protocol.cpp"),
  "steering media protocol source",
);
const wheelTargetSource = readRequired(
  path.join(codeDir, "control/wheel_target_mixer.hpp"),
  "wheel target bound source",
);
const lines = markdown.split(/\r?\n/);
const failures = [];
let checkedTableCount = 0;
let checkedValueCount = 0;
let checkedContractCount = 0;

function tableCells(line) {
  return line
    .trim()
    .replace(/^\|/, "")
    .replace(/\|$/, "")
    .split("|")
    .map((cell) => cell.trim());
}

function inlineCodeValues(cell) {
  return [...cell.matchAll(/`([^`]+)`/g)].map((match) => match[1]);
}

function resolveParameterPaths(parameterCell, lineNumber) {
  const names = inlineCodeValues(parameterCell);
  if (names.length === 0) {
    failures.push(`line ${lineNumber}: parameter cell has no inline-code path: ${parameterCell}`);
    return [];
  }

  const firstParent = names[0].includes(".")
    ? names[0].slice(0, names[0].lastIndexOf("."))
    : "";
  return names.map((name, index) => {
    if (index === 0 || name.includes(".") || firstParent === "") {
      return name;
    }
    return `${firstParent}.${name}`;
  });
}

function valueAtPath(root, dottedPath) {
  let value = root;
  for (const component of dottedPath.split(".")) {
    if (
      value === null ||
      typeof value !== "object" ||
      !Object.prototype.hasOwnProperty.call(value, component)
    ) {
      return { found: false };
    }
    value = value[component];
  }
  return { found: true, value };
}

function documentedValueMatches(documented, actual) {
  if (typeof actual === "number") {
    if (!/^-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(documented)) {
      return false;
    }
    return Number.isFinite(Number(documented)) && Object.is(Number(documented), actual);
  }
  if (typeof actual === "string") {
    return documented === actual;
  }
  if (typeof actual === "boolean") {
    return documented === String(actual);
  }
  return false;
}

function parseContractMarkers(text) {
  const markers = new Map();
  const markerPattern = /<!--\s*contract:([a-z0-9-]+)\s+([^\r\n]*?)\s*-->/g;
  for (const match of text.matchAll(markerPattern)) {
    const name = match[1];
    if (markers.has(name)) {
      failures.push(`duplicate structured contract marker: ${name}`);
      continue;
    }
    const attributes = new Map();
    for (const token of match[2].trim().split(/\s+/)) {
      const separator = token.indexOf("=");
      if (separator <= 0 || separator === token.length - 1) {
        failures.push(`contract ${name}: malformed attribute ${JSON.stringify(token)}`);
        continue;
      }
      const key = token.slice(0, separator);
      const value = token.slice(separator + 1);
      if (attributes.has(key)) {
        failures.push(`contract ${name}: duplicate attribute ${key}`);
      }
      attributes.set(key, value);
    }
    markers.set(name, attributes);
  }
  return markers;
}

function requireContract(markers, name, expectedAttributes) {
  const attributes = markers.get(name);
  checkedContractCount += 1;
  if (attributes === undefined) {
    failures.push(`missing structured contract marker: ${name}`);
    return;
  }
  for (const [key, expected] of Object.entries(expectedAttributes)) {
    const actual = attributes.get(key);
    if (actual !== expected) {
      failures.push(
        `contract ${name}: ${key} must be ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`,
      );
    }
  }
}

function requireSource(pattern, source, description) {
  const matched = typeof pattern === "string" ? source.includes(pattern) : pattern.test(source);
  if (!matched) {
    failures.push(`production source no longer proves ${description}`);
  }
}

for (let index = 0; index < lines.length; index += 1) {
  const header = tableCells(lines[index]);
  if (header[0] !== "参数" || header[1] !== "当前 JSON 值") {
    continue;
  }

  checkedTableCount += 1;
  let tableRowCount = 0;
  const separatorCells = tableCells(lines[index + 1] ?? "");
  if (
    separatorCells.length < 2 ||
    !/^:?-{3,}:?$/.test(separatorCells[0]) ||
    !/^:?-{3,}:?$/.test(separatorCells[1])
  ) {
    failures.push(`line ${index + 2}: current-value table has no valid separator row`);
  }
  index += 2;
  while (index < lines.length && lines[index].trim().startsWith("|")) {
    const cells = tableCells(lines[index]);
    const lineNumber = index + 1;
    const paths = resolveParameterPaths(cells[0] ?? "", lineNumber);
    const documentedValues = inlineCodeValues(cells[1] ?? "");
    tableRowCount += 1;

    if (paths.length !== documentedValues.length) {
      failures.push(
        `line ${lineNumber}: ${paths.length} parameter path(s) but ` +
          `${documentedValues.length} documented value(s)`,
      );
      index += 1;
      continue;
    }

    for (let valueIndex = 0; valueIndex < paths.length; valueIndex += 1) {
      const parameterPath = paths[valueIndex];
      const documented = documentedValues[valueIndex];
      const actualResult = valueAtPath(parameters, parameterPath);
      checkedValueCount += 1;

      if (!actualResult.found) {
        failures.push(`line ${lineNumber}: JSON path not found: ${parameterPath}`);
        continue;
      }
      const actual = actualResult.value;
      if (actual !== null && typeof actual === "object") {
        failures.push(`line ${lineNumber}: JSON path is not a scalar: ${parameterPath}`);
        continue;
      }
      if (!documentedValueMatches(documented, actual)) {
        failures.push(
          `line ${lineNumber}: ${parameterPath} documents ${JSON.stringify(documented)}, ` +
            `JSON has ${JSON.stringify(actual)}`,
        );
      }
    }

    index += 1;
  }

  if (tableRowCount === 0) {
    failures.push(`line ${index + 1}: current-value table has no data rows`);
  }
  index -= 1;
}

const contractMarkers = parseContractMarkers(markdown);
requireContract(contractMarkers, "params-load-fail-closed", {
  codes: "params.missing,params.parse,params.validation",
  out: "unchanged",
  startup: "refused",
});
requireSource('"params.missing"', paramStoreSource, "missing-file fail-closed diagnostics");
requireSource('"params.parse"', paramStoreSource, "parse fail-closed diagnostics");
requireSource('"params.validation"', paramStoreSource, "validation fail-closed diagnostics");
requireSource(
  /if \(!ReadText\([\s\S]*?"params\.missing"[\s\S]*?return false;/,
  paramStoreSource,
  "missing-file rejection",
);
requireSource(
  /if \(!ParseJsonObject\([\s\S]*?"params\.parse"[\s\S]*?return false;/,
  paramStoreSource,
  "JSON parse rejection",
);
requireSource(
  /if \(!required_ok \|\| optional_malformed\)[\s\S]*?"params\.validation"[\s\S]*?return false;/,
  paramStoreSource,
  "field and combination validation rejection",
);
const validationDiagnosticIndex = paramStoreSource.indexOf('"params.validation"');
const publishOutputIndex = paramStoreSource.indexOf("out = parsed;");
if (
  validationDiagnosticIndex < 0 ||
  publishOutputIndex < 0 ||
  publishOutputIndex <= validationDiagnosticIndex
) {
  failures.push(
    "production source no longer proves that validated parameters are published only after rejection paths",
  );
}

requireContract(contractMarkers, "reference-time-alignment-invalid", {
  action: "params.validation",
  out: "unchanged",
  startup: "refused",
});
requireSource(
  "ValidateReferenceTimeAlignmentParameters",
  referenceValidationSource,
  "reference time alignment combination validation owner",
);
requireSource(
  /if \(!ValidateReferenceTimeAlignment\([\s\S]*?optional_malformed = true;/,
  paramStoreSource,
  "reference time alignment invalid combinations feeding params.validation",
);

requireContract(contractMarkers, "yaw-path", {
  assistant: "yaw_control.valid/reason",
  media: "steering_snapshot.yaw_control.valid/reason",
});
requireSource(
  "std::string EncodeAssistantTelemetry",
  assistantProtocolSource,
  "assistant telemetry encoder owner",
);
requireSource(
  '\\"yaw_control\\"',
  assistantProtocolSource,
  "assistant root-level yaw_control JSON key",
);
requireSource(
  /telemetry\.yaw_control\.valid[\s\S]*?telemetry\.yaw_control\.reason/,
  assistantProtocolSource,
  "assistant yaw_control valid/reason fields",
);
requireSource(
  "void AppendSteeringSnapshotJson",
  mediaProtocolSource,
  "steering snapshot JSON encoder owner",
);
requireSource(
  '\\"yaw_control\\"',
  mediaProtocolSource,
  "steering snapshot yaw_control JSON key",
);
requireSource(
  /snapshot\.yaw_control\.valid[\s\S]*?snapshot\.yaw_control\.reason/,
  mediaProtocolSource,
  "steering snapshot yaw_control valid/reason fields",
);
requireSource(
  '\\"steering_snapshot\\"',
  mediaProtocolSource,
  "steering media steering_snapshot envelope",
);

requireContract(contractMarkers, "ml-enabled-speed-target", {
  range: "(0,5000]",
});
requireSource(
  /parsed\.ml\.maneuver\.speed_target > 0\.0/,
  paramStoreSource,
  "enabled ML speed target strict positive lower bound",
);
requireSource(
  /parsed\.ml\.maneuver\.speed_target <= control::kWheelSpeedTargetMax/,
  paramStoreSource,
  "enabled ML speed target inclusive production upper bound",
);
requireSource(
  /kWheelSpeedTargetMax\s*=\s*5000\.0/,
  wheelTargetSource,
  "shared 5000 wheel speed target upper bound",
);

requireContract(contractMarkers, "param-semantic-bounds", {
  ports: "[1,65535]",
  "control-period-ms": "[1,1000]",
  "runtime-window-max-ms": "86400000",
  "motion-turn-spinup": "finite[0,1]",
  "motion-stop-encoder": "[0,5000]",
  "low-voltage-threshold": "[1,INT_MAX]",
  "raw-turn-output": "[0,INT_MAX]",
});
requireContract(contractMarkers, "motion-confirmation-window", {
  "cycles-min": "1",
  "product-ms": "<=86400000",
});
requireSource(
  /kMinimumTcpPort\s*=\s*1/,
  runtimeParameterTypesSource,
  "shared TCP port lower bound declaration",
);
requireSource(
  /kMaximumTcpPort\s*=\s*65535/,
  runtimeParameterTypesSource,
  "shared TCP port upper bound declaration",
);
requireSource(
  /kMaximumRuntimeIntervalMs\s*=\s*24\s*\*\s*60\s*\*\s*60\s*\*\s*1000/,
  runtimeParameterTypesSource,
  "shared 24-hour runtime interval declaration",
);
requireSource(
  /kMaximumControlPeriodMs\s*=\s*1000/,
  runtimeParameterTypesSource,
  "control period upper bound declaration",
);
requireSource(
  /maximum_confirm_cycles[\s\S]*?kMaximumRuntimeIntervalMs\s*\/\s*parsed\.control_period_ms/,
  paramStoreSource,
  "motion confirmation cycles combined with the control period",
);
requireSource(
  /!std::isfinite\(parsed\.motion_turn_limit_spinup\)[\s\S]*?parsed\.motion_turn_limit_spinup < 0\.0[\s\S]*?parsed\.motion_turn_limit_spinup > 1\.0/,
  paramStoreSource,
  "motion spinup turn finite ratio bounds",
);
requireSource(
  /parsed\.motion_stop_encoder_threshold < 0[\s\S]*?control::kWheelSpeedTargetMax/,
  paramStoreSource,
  "motion stop encoder threshold bounds",
);

if (checkedTableCount === 0 || checkedValueCount === 0) {
  failures.push(
    `no current JSON values were checked (tables=${checkedTableCount}, values=${checkedValueCount})`,
  );
}

if (failures.length > 0) {
  console.error(
    `default_params documentation current-value check failed ` +
      `(${checkedTableCount} table(s), ${checkedValueCount} value(s)):\n` +
      failures.map((failure) => `- ${failure}`).join("\n"),
  );
  process.exit(1);
}

console.log(
  `default_params documentation static check passed (JSON values, markers, source/protocol shape only): ` +
    `${checkedTableCount} table(s), ${checkedValueCount} value(s), ` +
    `${checkedContractCount} structured contract(s); ` +
    `run run_default_params_documentation_contract_test.sh for the production loader contract`,
);

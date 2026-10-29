"""Build the DES-003 Rev 0.5 draft workbook from Rev 0.4 (openpyxl; the workbook has no drawings)."""
import copy
import re
import sys

import openpyxl
from openpyxl.worksheet.hyperlink import Hyperlink

SRC, DST = sys.argv[1], sys.argv[2]
wb = openpyxl.load_workbook(SRC)
OLD_HDR, NEW_HDR = "Rev 0.4  |  2026-10-22", "Rev 0.5 DRAFT  |  2026-10-29"

# ---- header line on every sheet ---------------------------------------------------------------
for ws in wb.worksheets:
    for row in ws.iter_rows(max_row=3):
        for c in row:
            if isinstance(c.value, str) and OLD_HDR in c.value:
                c.value = c.value.replace(OLD_HDR, NEW_HDR)

# ---- Function Register --------------------------------------------------------------------------
fr = wb["Function Register"]
COLS = {"id": 1, "level": 2, "parent": 3, "name": 4, "desc": 5, "input": 6, "control": 7, "output": 8, "mech": 9,
        "owner": 10, "phase": 12, "notes": 13, "sim": 14, "simnote": 15}
SW = "Weld planning software (IPC, Ubuntu + ROS2)"
WEB = "Web UI (awm_web) + backend (awm_api)"
ENG = "Process / robot engineer"
POS = "Rotary positioner (ROS 2, awm_positioner)"


def j(*parts):
    return "\n".join(parts)


rows = []
first, last = None, None
for r in range(5, fr.max_row + 1):
    fid = fr.cell(r, 1).value
    if fid and re.match(r"A\d+$", str(fid)):
        first = first or r
        last = r
        rows.append({k: fr.cell(r, c).value for k, c in COLS.items()})
by_id = {x["id"]: x for x in rows}
style_tpl = {lvl: [copy.copy(fr.cell(r, c)._style) for c in range(1, 16)]
             for lvl, r in ((1, first), (2, first + 1))}
assert by_id["A1"]["level"] == 1 and by_id["A11"]["level"] == 2


def upd(fid, **kw):
    row = by_id[fid]
    for k, v in kw.items():
        assert k in COLS, k
        row[k] = v
    old = row.get("notes") or ""
    if "notes_add" not in kw:
        pass


def note(fid, text):
    old = by_id[fid].get("notes") or ""
    by_id[fid]["notes"] = (old + "; " if old else "") + text


# A1 import CAD
upd("A1", desc="Read the customer STEP file (STL fallback in Phase 2), split solids, normalise units (mm), show the "
               "3D model; import the cell / fixture CAD (Fusion STEP assemblies) and place the part in the virtual "
               "cell.",
    input=j("D01 STEP file", "D21 Cell & fixture CAD", "D04 Seam list (feedback, placement study)"),
    control=j("Customer drawing & revision", "D16 Cell & runtime configuration"),
    output=j("D02 B-Rep model", "D03 Part record", "D14 Cell model (URDF)", "D19 Part placement & collision scene",
             "D20 3D view model", "D08 Calibration data (nominal)"))
note("A1", "Rev0.5: A17, A18 added; ICOM aligned with sub-functions")
upd("A13", output=j("D20 3D view model", "Part collision mesh"),
    desc="Tessellate and export the 3D view model D20: one mesh per B-Rep face (face_id), one polyline per B-Rep edge "
         "(edge_id, chord <= 0.05 mm) + topology (edge -> faces, tangency, fingerprint). Shown in the Web 3D simulator "
         "view (rotate, zoom, pick). Part collision mesh exported separately (chord <= 0.1 mm near seams, face_id per "
         "triangle) for D19.")
upd("A15", desc="Set the fixture -> part transform and build the collision scene D19 (part, fixture, clamps, holders "
                "on the positioner flange). Source 'fixture_cad': transform read from the fixture assembly (markers "
                "REF_FIXTURE / REF_PART) - required for a production release. Source 'study': candidate from the "
                "placement study (A17).",
    input=j("D02 B-Rep model", "Part collision mesh", "D14 Cell model (URDF)", "D21 Cell & fixture CAD",
            "Placement decision"),
    control=j("Fixture design (locating pins)", "UR5e reach 850 mm / payload 5 kg"),
    mech=j(SW, ENG))
note("A15", "Rev0.5: placement source study / fixture_cad (CP-22)")
upd("A2", input=j("D02 B-Rep model", "D20 3D view model"),
    control=j("Customer drawing (weld symbols)", "User accounts & permissions"))
upd("A21", desc="User clicks edges in the 3D simulator view (D20); the software chains tangent-continuous edges into "
                "one seam and resolves the IDs to the exact B-Rep curve. Done once per part number by the Engineer "
                "(only role allowed, decision 2026-10-27).",
    control=j("Customer drawing (weld symbols)", "User accounts & permissions"),
    mech=j(WEB, ENG))
note("A21", "Rev0.5: Engineer only (CP-21)")
upd("A22", desc="Joint type (fillet, butt, lap, groove, pipe-to-flange), closed (360 deg) or open, weld side, start "
                "point, direction (reverse), split into segments, weld sequence (symmetric order to limit "
                "distortion), leg size. Tack welds are made by hand by the operator after clamping (A81), not by the "
                "robot.")
note("A22", "Rev0.5: tacks by hand (CP-19)")
upd("A3", output=j("D06 Seam process parameters", "D05 WPS library"),
    control=j("Process selection rules", "Weld quality standard (TBD, OI-12)"))
upd("A4", desc="Turn seams into head poses with angles and focus offset, approach / retract, IK with redundant tool "
               "rotation and positioner angle (Modes C / B), collision check, speed timing and cable / joint limit "
               "checks. Uses the active calibration D08 (plan after calibration).",
    input=j("D04 Seam list", "D06 Seam process parameters", "D19 Part placement & collision scene",
            "D08 Calibration data"),
    control=j("D14 Cell model (URDF)", "D05 WPS library", "D16 Cell & runtime configuration",
              "Robot reach / joint limits"))
note("A4", "Rev0.5: D08 input (CP-18), positioner (CP-20)")
upd("A41", desc="Sample points by arc length, step 1-3 mm (finer where the curve bends sharply). Seam frame per "
                "point: X = tangent (travel), Z = bisector of the two face normals pointing out of the material (up, "
                "towards the head; butt joint between tubes: n1 = n2, Z = surface normal), Y = Z x X. Beam axis = -Z.")
note("A41", "Rev0.5: Z convention (CP-9)")
upd("A42", desc="Work angle (default = bisector), travel angle push / pull 5-15 deg, focus offset (default 0; TCP at "
                "the nozzle / wire tip, CP-10), overlap of a few mm at the closing point of 360 deg seams. Weaving is "
                "done by the head wobble (preset parameter), not by robot motion.")
upd("A43", desc="Approach / retract 30-50 mm along the beam axis (poses, before A44). Air moves between seams and to "
                "home are planned with OMPL after A44 / A45 from the chosen joint states; positioner index moves with "
                "the robot parked (A49). All moves checked densely (<= 0.02 rad joint step), clearance >= 10 mm.",
    input=j("Tool poses", "Joint path"),
    control=j("Clearance rules", "D16 Cell & runtime configuration"))
upd("A44", desc="Per point, sample the tool-axis rotation psi (10 deg steps) and the 8 UR5e IK branches (analytic IK; "
                "pick_ik is not used); drop invalid states; layered graph within one IK branch, joint step <= 5 deg per "
                "point, cost = joint change + deviation from nominal + low-clearance penalty + wrist-3 travel; pick a "
                "continuous chain (method of TST-002). Robot base from the active calibration D08.",
    input=j("Complete path", "D19 Part placement & collision scene", "D08 Calibration data", "Positioner angles"))
upd("A45", desc="Check robot, LightWELD head and mount (incl. fiber / wire-feed conduit near the head), positioner "
                "holders, fixture, clamps and part on meshes generated from STEP. Only the nozzle tip (last 5 mm) may "
                "touch the part (CP-10). Padding 3 mm, weld clearance >= 5 mm (head body, mount, conduit). Produce a "
                "collision list per point.",
    input=j("Joint path", "D19 Part placement & collision scene"),
    control=j("D14 Cell model (URDF)", "D16 Cell & runtime configuration (padding, clearances)"),
    output=j("Collision-checked joint path", "Collision list"))
note("A45", "Rev0.5: D07 now produced by A48 (end of the A4 chain)")
upd("A47", desc="Time-parameterize each weld segment so that the TCP speed relative to the PART equals the WPS travel "
                "speed (error < 5 %). Mode C: robot at rest, positioner speed = travel speed / seam radius.",
    input=j("Collision-checked joint path", "D06 Seam process parameters"),
    control=j("UR5e joint velocity / acceleration limits", "D16 Cell & runtime configuration"))
upd("A48", desc="Wrist-3: +/-270 deg absolute soft limit and +/-180 deg travel within one seam so the fiber and wire "
                "conduit never wind; keep fiber bend radius >= 50 mm; check joint-limit margin. Write D07: tool "
                "poses (mm, part frame), timed joint path incl. positioner, per-point status, D08 hash.",
    input=j("Timed joint trajectory", "Collision list"),
    output="D07 Weld path (part frame)")
note("A48", "Rev0.5: produces D07 (CP-15); two wrist-3 checks (CP-14 #3)")
upd("A49", desc="Choose the positioner mode and angles per seam (decision 2026-10-27, positioner in Phase 1, "
                "controlled by ROS 2): Mode C - seam coaxial with the positioner axis: robot holds one pose, the "
                "positioner turns during emission; Mode B - rotate, stop, weld: positioner angle per arc segment, "
                "searched together with psi and the IK branch (robot-only first). Continuous 7-axis motion (options "
                "A1 / A2, DES-001 6.3) stays in Phase 2.",
    input=j("Tool poses", "D19 Part placement & collision scene"),
    control=j("Positioner spec (OI-14)", "D16 Cell & runtime configuration (positioner limits)"),
    output="Positioner angles", phase="Phase 1", sim="Sim-1 MoveIt mock", simnote="Mock positioner joint")
note("A49", "Rev0.5: Phase 1 (CP-20, OI-20 closed)")
upd("A51", desc="Touch the nozzle / wire tip (TCP = contact point on the beam axis, CP-10) to a fixed sharp point "
                "from 4-6 orientations; least-squares solve tool0 -> TCP. Target repeatability < 0.3 mm.",
    output="D08 Calibration data (TCP)")
upd("A52", output="D08 Calibration data (fixture frame)")
upd("A54", output="Error budget report")
upd("A55", phase="Phase 1", sim="Stub", simnote="Nominal positioner axis from the cell CAD (A18)")
note("A55", "Rev0.5: Phase 1 (CP-20)")
upd("A6", input=j("D07 Weld path (part frame)", "D08 Calibration data", "D06 Seam process parameters",
                  "D03 Part record"))
upd("A61", desc="Apply base -> positioner axis -> Rz(theta) -> flange -> fixture -> part transforms and the TCP; reuse "
                "the joint path of D07 after an FK check (no new IK).")
upd("A62", desc="ROS2 joint-trajectory segments for the robot (6 joints) and the positioner (1 joint), replayed by "
                "ur_robot_driver and the positioner controller (ADR-01, ADR-11); URScript / Toolpath URCap export "
                "kept as fallback for robot-only programs.")
upd("A63", desc="Gas pre-flow, preset select, wire-feed ready, trigger -> emission, wait for laser-ready / emission "
                "feedback (timeout -> one retry -> fault), dwell; at the end power ramp-down (if supported), emission "
                "off, wire stop / retract, gas post-flow. Events only at segment boundaries while robot AND "
                "positioner are at rest; Mode C: WeldStart -> positioner weld segment -> WeldEnd.")
upd("A64", desc="Save in the program library with the part record and the calibration hash.")
upd("A7", input=j("D09 Robot program", "D07 Weld path (part frame)", "Error budget report"),
    control=j("D14 Cell model (URDF)", "D19 Part placement & collision scene", "D05 WPS library",
              "APLSMN601-REQ-002 Risk Assessment"))
upd("A71", desc="Sim-1: playback of robot + positioner in the Web view / RViz on mock hardware; Sim-2: replay on URSim. "
                "Dense collision check of every segment; reach, collisions, cycle time. Points in collision, out of "
                "reach or near a singularity are shown in red.",
    input=j("D09 Robot program", "D07 Weld path (part frame)"),
    control=j("D14 Cell model (URDF)", "D19 Part placement & collision scene"))
upd("A74", desc="Production release: PASS D15, dry run, trial weld, error budget, placement source fixture_cad; "
                "Engineer requests, Admin approves; program locked. Simulation-scope release: PASS D15 only, marked "
                "not valid for production; used by Sim-2.",
    input=j("D15 Simulation report", "Dry-run result", "Trial weld result", "Error budget report"),
    control=j("Approval by engineering", "User accounts & permissions"),
    sim="Stub", simnote="Simulation-scope release only (CP-17)")
note("A74", "Rev0.5: release scopes (ADR-17)")
upd("A8", control=j("D11 I/O & handshake signal list", "D16 Cell & runtime configuration",
                    "UR safety configuration"),
    output=j("Welded part", "D12 Weld log", "D17 Run recording", "D18 Part weld report"),
    mech=j("UR5e robot + controller", POS, "PLC / safety relay", "IPG LightWELD 1000 + wire feeder", "Operator"))
upd("A81", desc="Select the released job (release scope and calibration hash checked), load and clamp the parts, tack "
                "them by hand, start from outside the guard. A Start from the HMI / UI is only a request; motion "
                "needs the physical start button at the cell.",
    input=j("Workpiece", "D10 Released program", "D08 Calibration data"),
    control=j("Operation manual", "User accounts & permissions"))
note("A81", "Rev0.5: manual tacks (CP-19); job check")
upd("A82", desc="Ready, start, preset select, trigger / emission, laser ready, interlock loops, positioner drive ready "
                "/ fault (hard-wired), cycle end, alarm signals.")
upd("A83", desc="Execute the released program: robot segments through ur_robot_driver, positioner segments through "
                "its controller, laser welding by the PLC. Weld segments run only at 100 % speed (UR speed scaling; "
                "positioner in Mode C); any change during a weld stops the seam (FAULT).",
    mech=j("UR5e robot + controller", POS, "IPG LightWELD 1000 + wire feeder"),
    simnote="Motion through ur_robot_driver + URSim and a mock positioner joint, no laser")
upd("A84", desc="LightWELD fault, interlock open (door, external loops, workpiece-clamp contact, fiber), wire-feeder "
                "fault, gas fault, positioner drive fault, collision, speed-scaling change, E-stop, communication loss "
                "(IPC heartbeat / watchdog -> emission off).")
upd("A85", input=j("Handshake signals", "D10 Released program", "D03 Part record", "Alarm / stop"))
upd("A88", desc="Users, login and roles (Operator, Engineer, Admin). Seam picking and editing: Engineer only. Every "
                "command is checked against the role and the cell state by the backend before it reaches M8.")
note("A88", "Rev0.5: no 'Seam setup' permission (CP-21)")
upd("A8", input=j("D10 Released program", "D08 Calibration data", "Workpiece", "D03 Part record"))
upd("A9", input=j("Welded part", "D12 Weld log", "D17 Run recording", "D18 Part weld report"))

NEW = [
    {"id": "A17", "level": 2, "parent": "A1", "name": "Study part placement",
     "desc": "Before the fixture is designed: evaluate candidate part orientations / positions on the positioner "
             "with the path planner (A4) for all seams; rank by seam coverage and clearance; the chosen placement "
             "goes to mechanical design for the fixture (then A15 imports it). Sim-1: candidates entered by hand; "
             "automatic search later (R3).",
     "input": j("D04 Seam list", "D02 B-Rep model", "D14 Cell model (URDF)"),
     "control": j("UR5e reach / positioner range", "Fixture design rules"),
     "output": "Placement decision", "mech": j(SW, ENG, "Mechanical designer"), "owner": "TBD", "phase": "Phase 1",
     "notes": "Rev0.5: new (CP-22)", "sim": "Sim-1 MoveIt mock", "simnote": "Candidates entered by hand"},
    {"id": "A18", "level": 2, "parent": "A1", "name": "Import cell CAD",
     "desc": "Read the cell and fixture STEP assemblies exported from Fusion (component tree, names, placements, mm); "
             "apply the fixed CAD -> world axis rotation of the file; map components to roles from the CAD map "
             "(static, positioner rotating, fixture, part, head, mount, ignore); read frames from REF_* marker "
             "components; generate collision meshes (convex hulls for surface models). Refuse the import if a marker "
             "is missing or duplicated. The UR5e model itself comes from ur_description.",
     "input": "D21 Cell & fixture CAD",
     "control": j("CAD map and marker naming rule (DES-001 ADR-16)", "D16 Cell & runtime configuration"),
     "output": j("D14 Cell model (URDF)", "D08 Calibration data (nominal)"),
     "mech": j(SW, "Mechanical designer"), "owner": "TBD", "phase": "Phase 1",
     "notes": "Rev0.5: new (CP-23, ADR-16)", "sim": "Sim-1 MoveIt mock", "simnote": "Nominal frames from markers"},
]
for n in NEW:
    by_id[n["id"]] = n


def sort_key(fid):
    return (int(fid[1]), 0 if len(fid) == 2 else int(fid[2:]))


ordered = sorted(by_id.values(), key=lambda x: sort_key(x["id"]))

# placeholder row 5 ("Web UI - Later") removed: Web UI is decided for Phase 1 (ADR-09)
for c in range(1, 16):
    if fr.cell(5, c).value is not None and c != 11:
        fr.cell(5, c).value = None

K = ('=IF(A{r}="","",IF(B{r}=1,IF(COUNTIF($C$6:$C$302,A{r})=0,"Not started",IF(COUNTIFS($C$6:$C$302,A{r},'
     '$K$6:$K$302,"Done")=COUNTIF($C$6:$C$302,A{r}),"Done",IF(COUNTIFS($C$6:$C$302,A{r},$K$6:$K$302,"In progress")'
     '+COUNTIFS($C$6:$C$302,A{r},$K$6:$K$302,"Done")>0,"In progress","Not started"))),IFERROR(IF(T(INDEX(INDIRECT('
     '"\'"&C{r}&"\'!$M$20:$M$28"),VALUE(MID(A{r},LEN(C{r})+1,2))))="","Not started",T(INDEX(INDIRECT("\'"&C{r}&"\'!'
     '$M$20:$M$28"),VALUE(MID(A{r},LEN(C{r})+1,2))))),"Not started")))')
assert fr.cell(first, 11).value == K.format(r=first), "status formula pattern changed"
row_of = {}
for i, x in enumerate(ordered):
    r = first + i
    row_of[x["id"]] = r
    for k, c in COLS.items():
        cell = fr.cell(r, c)
        cell._style = copy.copy(style_tpl[x["level"]][c - 1])
        cell.value = x.get(k)
    fr.cell(r, 11)._style = copy.copy(style_tpl[x["level"]][10])
    fr.cell(r, 11).value = K.format(r=r)
print("function rows", first, "->", first + len(ordered) - 1)

# "Edit in Function Register" links on A1...A9 pointed to stale rows; point them at the parent row
for p in [f"A{i}" for i in range(1, 10)]:
    ws = wb[p]
    ws["D3"].hyperlink = Hyperlink(ref="D3", location=f"'Function Register'!A{row_of[p]}")

# ---- Data Items ----------------------------------------------------------------------------------
di = wb["Data Items"]
drow = {di.cell(r, 1).value: r for r in range(5, di.max_row + 1) if di.cell(r, 1).value}
DESC = {
    "D07": "Tool poses (mm, part frame), timed joint path incl. positioner angle and mode, per-point status, D08 hash",
    "D08": "TCP, fixture frame, part offset, positioner axis; NOMINAL from the cell CAD (A18) until measured",
    "D11": "PLC <-> robot <-> LightWELD signals, positioner drive ready / fault",
    "D14": "Robot (ur_description), LightWELD head + mount, positioner joint and holders, Pismo frame, fixture, clamps; "
           "generated from the cell CAD (D21) by A18",
    "D16": "Cell parameters: soft limits, positioner limits, speeds, timeouts, clearances (padding / weld / air), "
           "network, I/O mapping reference",
    "D19": "Fixture -> part transform, part orientation, collision scene objects; source study / fixture_cad",
}
for k, v in DESC.items():
    di.cell(drow[k], 3).value = v
for k in ("D09", "D10"):
    di.cell(drow[k], 4).value = "JSON: ROS 2 joint-trajectory segments (robot + positioner) + events; URScript fallback"
di.cell(drow["D10"], 3).value = "Approved, revision-locked program; scope simulation / production"
di.cell(drow["D14"], 5).value = "External: D21 cell CAD via A18"
r21 = drow["D20"] + 1
assert di.cell(r21, 1).value is None
for c in range(1, 9):
    di.cell(r21, c)._style = copy.copy(di.cell(drow["D20"], c)._style)
vals = ["D21", "Cell & fixture CAD",
        "Cell, fixture, head and positioner geometry from mechanical design, with marker components REF_*",
        "STEP AP214 / AP242 assembly (Fusion export, mm)", "External (mechanical design, Fusion)"]
for c, v in enumerate(vals, start=1):
    di.cell(r21, c).value = v
for c in (6, 7, 8):
    di.cell(r21, c).value = di.cell(drow["D20"], c).value.replace(f"{drow['D20']}", f"{r21}")

# ---- Revision History ---------------------------------------------------------------------------
rh = wb["Revision History"]
for c in range(1, 5):
    rh.cell(9, c)._style = copy.copy(rh.cell(8, c)._style)
rh["A9"], rh["B9"], rh["D9"] = "0.5 DRAFT", "2026-10-29", "Truong Hoang (draft by the software agent)"
rh["C9"] = ("Decisions of 2026-10-27 and document review of 2026-10-29. New A17 (placement study), A18 (import cell "
            "CAD), D21 (cell & fixture CAD). A49 / A55 to Phase 1 (positioner, ROS 2 control). IDEF0 flows closed: "
            "D07 produced by A48, D08 by A51 / A52 / A18, D14 by A18; error budget to A74; collision list to A48; "
            "parent ICOM = union of sub-functions. A21 / A88 Engineer-only seam picking; A22 / A81 manual tacks; "
            "A41 Z convention; A74 release scopes; D07 / D09 / D10 / D19 contents. Placeholder row 'Web UI - Later' "
            "removed. 'Edit in Function Register' links fixed. Sheet 'A0 Overview Phase 1' renamed to work packages "
            "P1-P4.")
tpl = {k: [copy.copy(rh.cell(r, c)._style) for c in range(1, 5)] for k, r in (("title", 11), ("head", 12), ("row", 13))}
start = 20
rh.cell(start, 1).value = "Decisions taken 2026-10-27 (Rev 0.5)"
for c in range(1, 5):
    rh.cell(start, c)._style = copy.copy(tpl["title"][c - 1])
for c, v in enumerate(["#", "Topic", "Decision", "Ref"], start=1):
    rh.cell(start + 1, c).value = v
    rh.cell(start + 1, c)._style = copy.copy(tpl["head"][c - 1])
DEC = [
    ("Axes", "Z positive up. Seam frame Z = bisector of the face normals out of the material; beam axis = -Z.",
     "DES-001 5.3"),
    ("TCP", "TCP at the nozzle / wire tip (contact point), focus offset default 0; nozzle tip only may touch the part. "
            "Assumption until IPG data.", "DES-001 5.3, 5.4, OI-13"),
    ("Positioner", "1-axis rotary positioner in Phase 1, controlled by ROS 2 (ros2_control); Modes C and B; "
                   "continuous 7-axis in Phase 2. Supersedes decision 1 of 2026-10-22.", "DES-001 ADR-11, 6.3"),
    ("Cell geometry", "Cell, fixture, head and positioner from Fusion STEP assemblies with REF_* marker components; "
                      "robot from ur_description.", "DES-001 ADR-16"),
    ("Release", "Release scopes simulation / production; Sim-1 ends at A71; Sim-2 replays a simulation release.",
     "DES-001 ADR-17"),
    ("Placement", "Seams first, placement study (A17), fixture designed, placement imported (A15, source "
                  "fixture_cad).", "A15, A17"),
    ("Calibration", "Path planning uses the active calibration; calibration change -> re-plan.", "A4, A44"),
    ("Tacks", "Tack welds by hand after clamping, not by the robot.", "A22, A81, OI-26"),
    ("Roles", "Seam picking by Engineer only.", "A21, A88, DES-001 7.3"),
    ("Golden part", "564082 3-part assembly (pipe nipple, solid, elbow) from the cell CAD.", "TST-002"),
]
for i, (t, d, ref) in enumerate(DEC, start=1):
    r = start + 1 + i
    for c, v in enumerate([str(i), t, d, ref], start=1):
        rh.cell(r, c).value = v
        rh.cell(r, c)._style = copy.copy(tpl["row"][c - 1])

# ---- A0 Overview ----------------------------------------------------------------------------------
ov = wb["A0 Overview"]
if ov["F5"].value == "Swap to A2.":
    ov["F5"].value = None
ov["F17"].value = "D08 Calibration data  →  A4, A6, A8"

# ---- A0 Overview Phase 1 -> team work packages P1-P4 --------------------------------------------
p1 = wb["A0 Overview Phase 1"]
p1["A1"] = ("Team work packages P1-P4 (Phase 1), mapped to the functions. Rev 0.5: renamed from A1-A4 (clash with "
            "function IDs), input STEP (not STL), pose convention stated (DES-001 5.3).")
p1["A2"] = "Work package"
WP = {
    3: ("P1 (A1, A2)", "STEP import -> seam list (part frame)",
        "D01 STEP file (golden part: 564082 assembly from the cell CAD D21)",
        "Customer drawing; axis convention DES-001 5.3",
        "D04 seams.json - per point: position X, Y, Z (mm) + seam frame quaternion (x, y, z, w); normals n1, n2",
        "awm_geometry / awm_cad / awm_seam"),
    5: ("P2 (A4)", "Seam list -> weld path (psi x IK-branch ladder, positioner Modes C / B)",
        "D04, D06, D19, D08", "D14 cell model, D16 limits",
        "D07 path.json - tool poses (mm + quaternion, part frame); joint path q1...q6 (deg) + positioner angle; IK "
        "branch (shoulder, elbow, wrist = +/-1) and psi per point",
        "awm_path (MoveIt2, analytic UR IK)"),
    7: ("P3 (A6, A71)", "Program + simulation", "D07, D08", "D14, D19",
        "D09 program.json, D15 report; playback in the Web view / RViz", "awm_program / awm_verify / awm_web"),
    9: ("P4 (A8)", "ROS 2 to robot and positioner (Sim-2 on URSim, then hardware)",
        "D10 released program (scope simulation in Sim-2)", "D11, D16, UR safety configuration",
        "Joint trajectories via ur_robot_driver + positioner controller; weld events to the PLC",
        "awm_executor / awm_cell / awm_positioner / awm_plc_bridge"),
}
for r, (a, b, c, d, f, g) in WP.items():
    p1[f"A{r}"], p1[f"B{r}"], p1[f"C{r}"], p1[f"D{r}"], p1[f"F{r}"], p1[f"G{r}"] = a, b, c, d, f, g
    p1[f"J{r}"] = "Phase 1"

wb.calculation.fullCalcOnLoad = True  # formulas have no cached values; Excel recalculates on open
wb.save(DST)
print("saved", DST)

---
name: Predictive Braking Cut-in Demo
description: Offline classroom comparison of Reactive and Predictive CV braking.
colors:
  paper: "#f1f0eb"
  ink: "#1c302f"
  muted: "#526360"
  line: "#d4d9d1"
  green: "#176c59"
  blue: "#247cba"
  orange: "#d78035"
  red: "#a83c30"
  white: "#fff"
  scene-bar: "#e5eade"
  playback: "#f7f8f4"
  road-surround: "#dce3d7"
  road: "#434f4d"
  focus: "#b66d27"
typography:
  body:
    fontFamily: '"Avenir Next", Avenir, "Segoe UI", sans-serif'
    fontSize: "14px"
  display:
    fontSize: "clamp(26px,2.25vw,40px)"
    fontWeight: 600
    lineHeight: 1.08
    letterSpacing: "-.035em"
  view-title:
    fontSize: "22px"
    fontWeight: 600
    letterSpacing: "-.03em"
  section-title:
    fontSize: "15px"
    fontWeight: 600
  metric:
    fontSize: "22px"
    fontWeight: 500
  button:
    fontSize: "12px"
    fontWeight: 600
rounded:
  workspace: "14px"
  control: "6px"
  action: "5px"
  swatch: "2px"
spacing:
  control-gap: "7px"
  field-row: "12px"
  field-column: "20px"
  panel-inset: "22px"
  lower-gap: "32px"
components:
  button-primary:
    backgroundColor: "{colors.green}"
    textColor: "{colors.white}"
    rounded: "{rounded.control}"
    padding: "8px 12px"
  button-secondary:
    backgroundColor: "{colors.white}"
    textColor: "{colors.ink}"
    rounded: "{rounded.control}"
    padding: "8px 12px"
  workspace:
    backgroundColor: "{colors.white}"
    rounded: "{rounded.workspace}"
---

# Design System: Predictive Braking Cut-in Demo

## Overview

This records the shipped local browser demo. Its sources are `src/pbs/demo/static/style.css`, `index.html`, and `app.js`. The guidance applies to this demo interface; scientific plots elsewhere in the repository do not inherit it.

The interface is a restrained comparison instrument: warm paper, dark labels, green controls, and a shared road scene. The systems receive equal visual weight. Animation is the main focus; playback, measurements, and scenario controls support explaining the response.

**Key Characteristics:**
- Equal comparison panels sharing one clock and camera center.
- Compact labels with tabular numbers and explicit units.
- Shallow vehicle depth and flat interface surfaces.

## Colors

Green identifies primary actions, CV headings, control values, and the CV chart series. Paper, white, ink, muted text, and fine neutral dividers establish hierarchy.

Blue identifies ego vehicles; orange identifies the cut-in agent. Canvas bodies use shaded variants. The prediction legend is orange; the canvas forecast uses pale amber (`#f3bc72`) against the dark road. Reactive uses purple and a dashed chart line; CV uses solid green. CSS legend purple (`#6f598f`) and canvas series purple (`#80669b`) are distinct source values.

Braking uses red text on a pale warm badge; finished state returns to paper and muted text. The declared `red` token remains from the stylesheet, but the brake badge uses `#9b2f25`, `#ffede6`, and `#e7b5a8` directly.

## Typography

Use the frontmatter's system font stack; there is no external font dependency. Canvas labels use `sans-serif` at 10px. DOM numbers use tabular figures. The fluid headline leads, followed by panel titles and large metrics, then section headings.

Supporting text is compact: metric labels and footer text are 10px, subtitles and field labels 11px, and controls primarily 12px. Keep units and the meaning of TTC visible alongside numbers.

## Layout

The centered main container has a maximum width of 1800px and default padding of `22px 40px 12px`. A bordered workspace contains the scene strip, equal road panels, shared legend, and playback. Each panel has four metric columns: speed, TTC, center distance, and brake events with cumulative command time. Default desktop roads are 200px high.

The lower grid uses `1.15fr 1fr`, a 32px gap, and an 18px top margin. Scenario controls have a left divider and 30px inset; fields use two columns with `12px 20px` gaps.

At 1700px and above, top padding and spacing grow; roads become 240px high and the chart 140px. At 1000px and below, gutters and control gaps tighten and the desktop legend note hides. At 760px and below, comparisons and lower sections stack, roads become 220px high, playback wraps with a full-width scrubber, and the parameter divider moves above the form. The header note hides at this narrow breakpoint.

## Elevation & Depth

Interface surfaces are flat: borders, tonal strips, and whitespace separate regions. Canvas vehicles alone use shadows (`#17252150`, blur 7, offset 3px horizontally and 5px vertically), a lower body, windows, and lights. This is illustrative depth, not a change to simulation geometry.

## Shapes

Use the frontmatter's gently rounded workspace, controls, action badges, and legend swatches. The workspace clips its contents. Thin borders divide panels and metric groups. There is no floating-card shadow vocabulary.

## Components

### Buttons and fields

Green primary buttons drive Play and Run simulation. Replay and Step use white secondary buttons. Hover uses `filter: brightness(.94)`. Disabled buttons use opacity `.45` and a wait cursor. Buttons, selects, and inputs share a 3px amber focus outline with a 3px offset. Native sliders and checkboxes use the green accent.

Parameter fields show values and units, mark edits as Unapplied changes, and retain old results until Run simulation succeeds. Running disables relevant controls and changes the submit label to Simulating…. Errors appear in an alert region.

### Comparison and playback

Action badges explicitly read CRUISE, BRAKE, or FINISHED. Metrics show ego speed, estimated TTC, center distance, and brake event count with cumulative command time. End summaries offer a collapsible brake/release event table. A shared timeline, clock, playback speed, replay, and 0.1 s step control synchronize both views.

Positions and speed interpolate. At first contact the affected panel freezes its vehicles, camera and metrics; the other panel continues independently. Each title shows the observed first-brake time and panel time. Decisions, TTC, center distance, predictions, and chart samples follow the previous 0.1 s simulation sample. The forecast toggle affects only the CV overlay. The summary appears at the end and uses the same columns for both systems.

## Do's and Don'ts

### Do:
- **Do** retain equal comparison panels, shared time, and explicit metric units.
- **Do** preserve keyboard focus visibility and text labels for actions and errors.
- **Do** keep vehicle colors consistent across both scenes and their legend.
- **Do** distinguish smooth playback from the discrete decision interval.

### Don't:
- **Don't** replace the compact comparison with decorative marketing panels.
- **Don't** use the 2.5D illustration to imply physical collision dynamics.
- **Don't** treat a TTC dash as a guarantee of future safety.

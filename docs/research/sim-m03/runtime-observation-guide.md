# Reading native component diagnostics

This guide describes the existing read-only Viewer presentation. It does not
establish completion of SIM-M03, equipment calibration or a successful encounter.

## Start with identity and time

Check the selected entity and generation before comparing panels. A recycled
entity ID does not identify the same instance. A changed selection must receive
its matching acknowledgement before previous-instance data can be considered
current.

Read sample, processing and delivery times separately. Sample age describes the
observation; delivery age describes transport. Paused simulation time and wall
time are different clocks. Historical data can explain a prior event, but must
not be read as a current observation.

## Read availability before numbers

Unavailable means that the source did not provide a usable value. It does not
mean zero, an empty scene, no activity or a healthy component. An available
measurement can still be historical. Read the native cause alongside its
availability and validity interval.

Compare quantities only when their units and epochs agree. Display colors and
automatic color ranges are presentation choices; a color change alone does not
establish a change in a physical quantity. Use the numerical value and native
units shown by the panel.

## Follow observations across panels

The window groups existing native observations into images and labels,
spectra and detection records, estimated state and uncertainty, requested and
achieved response, energy and propulsion state, and native events. These are
different observations with independent availability, sampling and delivery.
An entry in one panel does not fill a missing value in another.

Use native event timestamps and instance identities when following a sequence.
A visible effect, a nearby trajectory or a graph extremum is not evidence of
a native hit, a component failure or mission success. Those require their own
native event or outcome records.

## Recording and replay

Keep a recording and its optional diagnostic companion together. Replay joins
recorded samples by their exact native simulation time. A missing sample is
unavailable; the Viewer must not interpolate an image or reconstruct an
unrecorded component response. A checksum detects corrupted records; it does
not authenticate their origin or prove physical accuracy.

## Verification evidence

The [archived checkpoint evidence](../../../artifacts/sim-m03/verification/checkpoint106/README.md)
retains the latest inspected build and focused-test logs. It separates the
earlier Viewer build/tests from the later SDK checks. Neither set contains a
current full-screen visual acceptance record or full-program acceptance.

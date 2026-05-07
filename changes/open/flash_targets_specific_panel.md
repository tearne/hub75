# Flashing should target a specific panel when multiple are attached

## Intent

When more than one HUB75 panel is plugged into the host, flashing currently has no safe way to target a chosen panel — the flashing tool picks one of the attached devices, which may not be the intended one. This already caused real damage: a USB cable failed mid-test, a different panel was reflashed by accident, and because the panels are physically different types the wrong firmware caused cascading problems.

Establish the convention this project will use to identify panels (e.g. a serial number, a user-set name, a slot tag — TBD), and use it to make flashing target a specific panel deterministically. This change is the one that decides the identity scheme; later changes (notably client-side targeting) will consume it.

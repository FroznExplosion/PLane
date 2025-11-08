# Camera Controls Quick Reference

## Cycling Camera Modes

Press **C** to cycle through camera modes:

1. **CHASE** - Standard third-person chase camera
2. **MOUSE_CHASE** - 🆕 Mouse-controlled free look
3. **COCKPIT** - First-person cockpit view
4. **ORBIT** - Circular orbit camera
5. **CINEMATIC** - Dramatic angles

---

## Mouse Chase Camera (New!)

### Activation
1. Press **C** until you see "Camera mode: MOUSE_CHASE"
2. You're now in mouse look mode

### Controls
| Input | Action |
|-------|--------|
| **Right Mouse Button** (hold) | Enable free look |
| **Move Mouse** (while holding RMB) | Rotate camera around aircraft |
| **Release RMB** | Camera auto-centers behind aircraft |

### Features
- ✅ **360° rotation** - Look at aircraft from any angle
- ✅ **Smooth movement** - Camera follows aircraft smoothly
- ✅ **Auto-center** - Automatically returns to default view
- ✅ **Pitch limiting** - Camera won't flip upside down

### Tips
- Great for **inspecting your aircraft** during flight
- Perfect for **cinematic replays**
- Useful for **checking six o'clock** (behind you)
- Hold RMB and look around during maneuvers

---

## All Camera Modes Explained

### 1. CHASE (Default)
- Third-person view
- Follows behind and above aircraft
- Always faces forward
- Best for **normal flying**

### 2. MOUSE_CHASE (New!)
- Third-person view
- **Right-click to free look**
- Orbits around aircraft
- Best for **looking around**

### 3. COCKPIT
- First-person view
- Inside the aircraft
- Realistic pilot perspective
- Best for **immersive flight**

### 4. ORBIT
- Automatic circular orbit
- Always shows side of aircraft
- Slowly rotates around
- Best for **watching yourself fly**

### 5. CINEMATIC
- Dynamic camera angles
- Smooth dramatic movements
- Slight side-to-side drift
- Best for **screenshots/videos**

---

## Customizing Mouse Camera

You can adjust these settings in the Inspector (select Camera3D node):

### Mouse Chase Camera Settings
- **Mouse Sensitivity**: How fast camera rotates (default: 0.3)
- **Mouse Distance**: How far from aircraft (default: 20m)
- **Mouse Smoothing**: Camera movement smoothness (default: 6.0)
- **Mouse Pitch Limit**: Max up/down angle (default: 85°)
- **Mouse Auto Center Speed**: How fast camera returns (default: 0.5)

### Example Adjustments

**More sensitive mouse**:
```
mouse_sensitivity = 0.5
```

**Closer camera**:
```
mouse_distance = 15.0
```

**Faster auto-center**:
```
mouse_auto_center_speed = 1.0
```

---

## Troubleshooting

### Mouse look doesn't work
- Make sure you're in MOUSE_CHASE mode (press C)
- Check you're **holding right mouse button**
- Verify mouse is moving while RMB is held

### Camera too sensitive
- Decrease `mouse_sensitivity` in Inspector
- Lower values = slower rotation

### Camera too slow
- Increase `mouse_sensitivity`
- Decrease `mouse_smoothing`

### Camera won't center
- Release right mouse button
- Increase `mouse_auto_center_speed`
- Or press C to switch to regular CHASE mode

---

## Mouse Look in Different Camera Modes

| Mode | Mouse Look |
|------|-----------|
| CHASE | ❌ No mouse look |
| MOUSE_CHASE | ✅ Right-click to enable |
| COCKPIT | ❌ No mouse look (fixed to aircraft) |
| ORBIT | ❌ No mouse look (automatic rotation) |
| CINEMATIC | ❌ No mouse look (automatic) |

---

## Keyboard Summary

| Key | Action |
|-----|--------|
| **C** | Cycle camera mode |
| **Right Mouse Button** | Free look (in MOUSE_CHASE mode) |

---

## Pro Tips

### Combat/Dogfighting
Use **MOUSE_CHASE** to:
- Check if enemy is still behind you
- Track targets during turns
- See missile trails

### Formation Flying
Use **CHASE** or **MOUSE_CHASE**:
- Keep wingman in view
- Monitor aircraft spacing

### Landing
Use **CHASE**:
- See aircraft orientation clearly
- Judge altitude above runway

### Screenshots
Use **MOUSE_CHASE** or **CINEMATIC**:
- Find best angle
- Capture dramatic moments

### Learning
Use **COCKPIT**:
- Understand pilot perspective
- Practice instrument flying

---

## Future Camera Features (Coming Soon)

- [ ] Track-IR / head tracking support
- [ ] Custom camera positions
- [ ] Replay camera paths
- [ ] Missile camera view
- [ ] Ground-based camera
- [ ] Flyby camera
- [ ] Photo mode with free camera

---

**Enjoy your new camera freedom!** 🎥✈️

# Waldo Commander

A web interface for controlling robotic arms, currently tested with the [PAROL6](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm) robot.

https://github.com/user-attachments/assets/61c5aec4-9611-4f61-b1b0-35f25931e11e

- **Browser-based.** Control from any device on the network without being tethered to the arm.
- **Python programs.** Write robot programs in Python with loops, math, and libraries. Built-in editor with auto-complete, live output, and step-through debugging.
- **3D simulation.** Preview motion paths, check reachability, and scrub through the timeline — no physical robot needed.
- **Teach by demonstration.** Control the robot live and record the motions as Python code.
- **AI control (MCP).** A built-in [Model Context Protocol](https://modelcontextprotocol.io) server lets an LLM (Claude Code, Claude Desktop, …) read status, author and run programs, and drive the arm — with per-mode approval you control. See [AI Control (MCP)](https://jepson2k.github.io/Waldo-Commander/guides/mcp/).
- **Backend-agnostic.** Robot-specific logic lives behind the [waldoctl](https://github.com/Jepson2k/waldoctl) abstraction layer. Other robots can be integrated by implementing the same interfaces — see the [Backend Development Guide](https://jepson2k.github.io/Waldo-Commander/guides/backend-development/).

## Quick start

```bash
git clone https://github.com/Jepson2k/Waldo-Commander.git
cd Waldo-Commander
pip install -e ".[parol6]"
waldo-commander
```

Open the printed URL. No robot connected? The app auto-starts in simulator mode so you can explore.

For connecting hardware, platform-specific setup, and configuration, see [Getting Started](https://jepson2k.github.io/Waldo-Commander/getting-started/).

For the 5800X PAROL6 ZDT operator workflow, see
[`docs/PAROL6-ZDT操作员坐标恢复与Home.md`](docs/PAROL6-ZDT操作员坐标恢复与Home.md).
After a power cycle, place the arm near the signed Home and click **不运动坐标重建** first.
That button restores multi-turn coordinates from the current `0x31` within-turn reading and
`0x36` turn selection without moving the arm. Click **Home** only when the arm should actively
MoveJ to the exact canonical target. The 3D scene applies a J2-only `-20°` display offset;
readouts and motion commands remain canonical.

## Links

- [Documentation](https://jepson2k.github.io/Waldo-Commander/)
- [waldoctl](https://github.com/Jepson2k/waldoctl) — robot backend abstraction layer
- [PAROL6 hardware](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm)

## Safety

- This software provides no safety guarantees and assumes no liability
- User accepts full responsibility for robot operation
- Simulator mode is not physics-accurate and does not guarantee repeatability on real hardware
- The digital E-STOP is not a substitute for the hardware emergency stop
- Incorrect kinematics calculations could result in sudden robotic movements
- Keep clear of all moving parts during operation

## License

See [LICENSE](LICENSE).

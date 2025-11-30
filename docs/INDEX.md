# 📚 Documentation Index

Quick navigation to all project documentation.

---

## 🚀 Getting Started

Start here if you're new to the project:

1. **[README.md](../README.md)** - Project overview, features, and hardware requirements
2. **[QUICK_START.md](QUICK_START.md)** - Get flying in 30 minutes
3. **[PROJECT_COMPLETE.md](../PROJECT_COMPLETE.md)** - Project summary and completion status

---

## 🔧 Hardware Setup

Everything you need to build the hardware:

1. **[PARTS_LIST.md](PARTS_LIST.md)** - Complete bill of materials with prices and shopping links
2. **[WIRING_GUIDE.md](WIRING_GUIDE.md)** - Step-by-step wiring instructions with diagrams
3. **[PIN_CONFIGURATION.md](PIN_CONFIGURATION.md)** - Complete pin mapping reference

---

## 💻 Software Setup

Getting the code running:

1. **[LIBRARIES.md](LIBRARIES.md)** - Library installation instructions
2. **[Code Files](#code-files)** - Arduino sketches for RC and FC

---

## ⚙️ Configuration & Tuning

Optimizing your drone:

1. **[PID_TUNING.md](PID_TUNING.md)** - Complete PID tuning guide for stable flight
2. **[SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)** - Visual diagrams and architecture

---

## 🐛 Troubleshooting & Support

When things don't work:

1. **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions
2. **[PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)** - Code organization reference

---

## 📖 Reference Documents

Additional information:

1. **[CHANGELOG.md](../CHANGELOG.md)** - Version history and updates
2. **[LICENSE](../LICENSE)** - MIT License and safety disclaimer

---

## 📁 Code Files

### Remote Controller
- **Location**: `/RemoteController/RemoteController.ino`
- **Lines**: 522
- **Purpose**: Wireless remote control with joysticks, buttons, and switches

### Flight Controller
- **Location**: `/FlightController/FlightController.ino`
- **Lines**: 929
- **Purpose**: Quadcopter flight controller with PID stabilization

---

## 📊 Documentation Statistics

| Category | Files | Total Size |
|----------|-------|------------|
| Code Files | 2 | 38.8 KB |
| Documentation | 11 | 97.3 KB |
| Total Project | 14 files | 136.1 KB |

**Lines of Code**: 1,451 (522 RC + 929 FC)

---

## 🎯 Quick Reference by Task

### "I want to build the drone"
1. Read [PARTS_LIST.md](PARTS_LIST.md)
2. Order components
3. Follow [WIRING_GUIDE.md](WIRING_GUIDE.md)
4. Upload code
5. Follow [QUICK_START.md](QUICK_START.md)

### "I want to tune my drone"
1. Read [PID_TUNING.md](PID_TUNING.md)
2. Modify PID values in FlightController.ino
3. Test and iterate

### "Something isn't working"
1. Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
2. Verify wiring in [PIN_CONFIGURATION.md](PIN_CONFIGURATION.md)
3. Check serial monitor output

### "I want to understand the code"
1. Read [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)
2. View [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
3. Review code comments

### "I want to modify/extend the project"
1. Understand [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
2. Review [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)
3. Check [CHANGELOG.md](../CHANGELOG.md) for planned features

---

## 📱 Print-Friendly Guides

These documents are optimized for printing:

- **[QUICK_START.md](QUICK_START.md)** - 7 pages
- **[WIRING_GUIDE.md](WIRING_GUIDE.md)** - 15 pages
- **[PIN_CONFIGURATION.md](PIN_CONFIGURATION.md)** - 7 pages
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 12 pages

**Tip**: Print these for reference while building!

---

## 🎓 Learning Path

### Beginner
1. Start with [README.md](../README.md) to understand the project
2. Review [PARTS_LIST.md](PARTS_LIST.md) to know what you need
3. Follow [QUICK_START.md](QUICK_START.md) step-by-step

### Intermediate
1. Study [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) to understand data flow
2. Read [PID_TUNING.md](PID_TUNING.md) to optimize flight
3. Review [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) to understand code

### Advanced
1. Modify PID algorithms in code
2. Add sensors (GPS, barometer, etc.)
3. Implement new features (altitude hold, return-to-home)
4. Share improvements with community

---

## 🔍 Search by Topic

### Hardware
- **NRF24L01**: [LIBRARIES.md](LIBRARIES.md), [WIRING_GUIDE.md](WIRING_GUIDE.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **MPU6050**: [LIBRARIES.md](LIBRARIES.md), [WIRING_GUIDE.md](WIRING_GUIDE.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Motors/ESCs**: [WIRING_GUIDE.md](WIRING_GUIDE.md), [QUICK_START.md](QUICK_START.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Joysticks**: [WIRING_GUIDE.md](WIRING_GUIDE.md), [PIN_CONFIGURATION.md](PIN_CONFIGURATION.md)

### Software
- **PID Control**: [PID_TUNING.md](PID_TUNING.md), [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
- **Communication**: [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md), [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)
- **Calibration**: [QUICK_START.md](QUICK_START.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

### Safety
- **Safety Features**: [README.md](../README.md), [LICENSE](../LICENSE)
- **Failsafe**: [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
- **Emergency Procedures**: [QUICK_START.md](QUICK_START.md)

---

## 📞 Support & Help

### Before Asking for Help

1. ✅ Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for your issue
2. ✅ Verify wiring using [WIRING_GUIDE.md](WIRING_GUIDE.md)
3. ✅ Review Serial Monitor output
4. ✅ Test components individually
5. ✅ Check component specifications match requirements

### Providing Information

If you need help, include:
- Which board (FC or RC)
- Error messages from Serial Monitor
- What you've already tried
- Hardware specifications
- Photos of wiring (if hardware issue)

---

## 🌟 Recommended Reading Order

### For Building
1. [README.md](../README.md) - Overview
2. [PARTS_LIST.md](PARTS_LIST.md) - Shopping
3. [LIBRARIES.md](LIBRARIES.md) - Software setup
4. [WIRING_GUIDE.md](WIRING_GUIDE.md) - Assembly
5. [QUICK_START.md](QUICK_START.md) - First flight

### For Learning
1. [README.md](../README.md) - Overview
2. [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) - Architecture
3. [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) - Code organization
4. [PID_TUNING.md](PID_TUNING.md) - Control theory
5. Review actual code files

### For Troubleshooting
1. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Solutions
2. [PIN_CONFIGURATION.md](PIN_CONFIGURATION.md) - Verify wiring
3. [WIRING_GUIDE.md](WIRING_GUIDE.md) - Check connections
4. [QUICK_START.md](QUICK_START.md) - Verify procedure

---

## 📋 Checklist Documents

Each guide includes checklists:

- ✅ **[WIRING_GUIDE.md](WIRING_GUIDE.md)** - Wiring verification checklist
- ✅ **[QUICK_START.md](QUICK_START.md)** - Pre-flight safety checklist
- ✅ **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Diagnostic checklist
- ✅ **[PARTS_LIST.md](PARTS_LIST.md)** - Shopping checklist

---

## 🎯 Document Purposes at a Glance

| Document | Purpose | When to Use |
|----------|---------|-------------|
| **README.md** | Project overview | First time learning about project |
| **QUICK_START.md** | Fast setup guide | Ready to build and fly |
| **PARTS_LIST.md** | Shopping guide | Before ordering parts |
| **WIRING_GUIDE.md** | Assembly instructions | During hardware assembly |
| **LIBRARIES.md** | Software setup | Before uploading code |
| **PIN_CONFIGURATION.md** | Pin reference | During wiring or troubleshooting |
| **PID_TUNING.md** | Flight optimization | After first flight |
| **TROUBLESHOOTING.md** | Problem solving | When something doesn't work |
| **SYSTEM_ARCHITECTURE.md** | Technical diagrams | Understanding system design |
| **PROJECT_STRUCTURE.md** | Code reference | Modifying or extending code |
| **CHANGELOG.md** | Version history | Checking updates |
| **LICENSE** | Legal info | Understanding terms |

---

## 💾 Offline Access

All documentation is in **Markdown format** and can be:
- Viewed in any text editor
- Converted to PDF using pandoc
- Printed for reference
- Read without internet connection

**Recommended**: Save entire `/docs` folder for offline reference.

---

## 🔄 Keeping Updated

Check [CHANGELOG.md](../CHANGELOG.md) for:
- New features
- Bug fixes
- Documentation updates
- Version history

---

**Last Updated**: 2025-11-30

**Total Documentation**: 12 files, 97+ KB, covering every aspect of the project

**Navigate**: Use links above to jump directly to any document!

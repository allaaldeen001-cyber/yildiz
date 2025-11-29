# Contributing to Professional Arduino Nano Drone System

Thank you for your interest in contributing to this project! This document provides guidelines for contributions.

---

## 🎯 Ways to Contribute

### 1. Code Improvements
- PID tuning optimizations
- Additional flight modes
- Performance enhancements
- Code refactoring

### 2. Documentation
- Improve existing guides
- Add translations
- Create video tutorials
- Add photos/diagrams

### 3. Bug Reports
- Report issues via GitHub Issues
- Include detailed reproduction steps
- Provide serial monitor output
- Attach photos of wiring if relevant

### 4. Feature Requests
- Suggest new features
- Propose improvements
- Share use cases

### 5. Hardware Variants
- Support for different boards (ESP32, STM32, etc.)
- Alternative sensor options
- Different frame configurations

---

## 📝 Contribution Guidelines

### Code Style

**Follow these conventions:**

```cpp
// Constants: UPPERCASE_WITH_UNDERSCORES
#define MAX_THROTTLE 2000

// Variables: camelCase
float gyroRate[3];
bool systemArmed;

// Functions: camelCase
void calibrateGyro() {
  // Function implementation
}

// Classes: PascalCase
class PIDController {
  // Class implementation
};
```

**Comments:**
- Use clear, descriptive comments
- Explain WHY, not just WHAT
- Document complex algorithms
- Keep comments up to date

**Indentation:**
- 2 spaces per indent level (no tabs)
- Opening brace on same line for functions
- Consistent spacing

### Commit Messages

Use clear, descriptive commit messages:

**Good:**
```
Add altitude hold feature using barometer
Fix PID integral windup issue
Update calibration guide with photos
```

**Bad:**
```
Update
Fix bug
Changed stuff
```

### Testing

Before submitting:
- [ ] Code compiles without errors
- [ ] Code compiles without warnings
- [ ] Tested on actual hardware
- [ ] No regression in existing features
- [ ] Updated documentation if needed

---

## 🔄 Pull Request Process

1. **Fork the repository**
   - Click "Fork" on GitHub
   - Clone your fork locally

2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make your changes**
   - Write clean, documented code
   - Test thoroughly
   - Update documentation

4. **Commit your changes**
   ```bash
   git add .
   git commit -m "Add feature: description"
   ```

5. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Create Pull Request**
   - Go to original repository
   - Click "New Pull Request"
   - Select your feature branch
   - Describe changes clearly

7. **Wait for review**
   - Respond to feedback
   - Make requested changes
   - Be patient and respectful

---

## 🐛 Bug Report Template

When reporting bugs, include:

```markdown
**Description:**
Clear description of the bug

**Steps to Reproduce:**
1. Step one
2. Step two
3. Step three

**Expected Behavior:**
What should happen

**Actual Behavior:**
What actually happens

**Serial Monitor Output:**
```
Paste serial monitor output here
```

**Hardware:**
- Arduino: Nano ATmega328P
- NRF24L01: PA+LNA version
- Motors: 1000KV
- ESC: 30A SimonK
- Battery: 3S 2200mAh

**Software:**
- Arduino IDE version: 1.8.19
- RF24 Library version: 1.4.2

**Photos/Videos:**
(if applicable)

**Additional Context:**
Any other relevant information
```

---

## ✨ Feature Request Template

```markdown
**Feature Description:**
Clear description of proposed feature

**Use Case:**
Why is this feature needed?
Who will benefit?

**Proposed Implementation:**
How should it work?
Any technical details?

**Alternatives Considered:**
Other solutions you've thought about

**Additional Context:**
Mockups, diagrams, references
```

---

## 🏗️ Development Setup

### Required Software:
- Arduino IDE 1.8.13+ or Arduino IDE 2.0+
- Git
- Text editor (VS Code, Sublime, etc.)

### Recommended Libraries for Testing:
```
RF24 by TMRh20
Wire (built-in)
```

### Hardware for Testing:
Minimum test setup:
- 2x Arduino Nano
- 2x NRF24L01 PA+LNA
- 2x 10μF capacitors
- 1x MPU6050
- Breadboard and jumpers

---

## 🔬 Areas Needing Contribution

### High Priority:
- [ ] Altitude hold implementation (barometer integration)
- [ ] Battery voltage monitoring
- [ ] Improved ESC control using Servo library
- [ ] Automatic PID tuning
- [ ] Return-to-home feature

### Medium Priority:
- [ ] GPS integration
- [ ] FPV camera integration guide
- [ ] Telemetry logging (SD card)
- [ ] Mobile app for configuration
- [ ] 3D printable remote controller case

### Documentation:
- [ ] Video assembly guide
- [ ] Step-by-step photo tutorial
- [ ] Translations (Spanish, Chinese, German, French)
- [ ] Advanced PID tuning guide
- [ ] Aerobatic flight modes

### Hardware Variants:
- [ ] ESP32 port (WiFi telemetry)
- [ ] STM32 port (more processing power)
- [ ] Support for different IMUs (MPU9250, BMI088)
- [ ] Support for different radios (HC-12, LoRa)

---

## 🎓 Learning Resources

If you're new to drone development:

**Arduino:**
- [Arduino Reference](https://www.arduino.cc/reference/en/)
- [Arduino Playground](https://playground.arduino.cc/)

**Drone Concepts:**
- [PID Control Explained](https://en.wikipedia.org/wiki/PID_controller)
- [Quadcopter Dynamics](https://www.youtube.com/watch?v=hGcGPUqB67Q)

**Embedded Systems:**
- [Interrupt Handling](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
- [I2C Communication](https://www.arduino.cc/en/Reference/Wire)

---

## 📜 Code of Conduct

### Our Pledge

We pledge to make participation in this project a harassment-free experience for everyone, regardless of:
- Age, body size, disability
- Ethnicity, gender identity
- Experience level
- Nationality, personal appearance
- Race, religion, sexual identity and orientation

### Our Standards

**Positive behavior:**
- Using welcoming and inclusive language
- Being respectful of differing viewpoints
- Gracefully accepting constructive criticism
- Focusing on what is best for the community
- Showing empathy towards other community members

**Unacceptable behavior:**
- Trolling, insulting/derogatory comments
- Public or private harassment
- Publishing others' private information
- Other conduct which could reasonably be considered inappropriate

### Enforcement

Violations may result in:
1. Warning
2. Temporary ban
3. Permanent ban

---

## 🙏 Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Mentioned in release notes
- Credited in documentation updates

---

## ❓ Questions?

- Open a GitHub Discussion
- Check existing Issues
- Review documentation

---

## 📄 License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file).

---

**Thank you for helping make this project better! 🚁**


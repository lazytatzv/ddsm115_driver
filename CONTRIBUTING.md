# Contributing to DDSM115 Driver Library

Thank you for your interest in contributing to the DDSM115 Driver Library! This document provides guidelines and information for contributors.

## Code of Conduct

This project adheres to a code of conduct that we expect all contributors to follow. Please be respectful and constructive in all interactions.

## How to Contribute

### Reporting Bugs

If you find a bug, please create an issue with:

1. **Clear description** of the problem
2. **Steps to reproduce** the issue
3. **Expected vs. actual behavior**
4. **Environment details**:
   - OS and version
   - Compiler version
   - Boost version
   - Hardware setup (if applicable)
5. **Error messages** or logs (if any)

### Suggesting Features

For feature requests, please:

1. **Check existing issues** to avoid duplicates
2. **Describe the use case** and motivation
3. **Provide examples** of how the feature would be used
4. **Consider implementation** complexity and compatibility

### Pull Requests

1. **Fork the repository** and create a feature branch
2. **Follow coding standards** (see below)
3. **Add tests** for new functionality
4. **Update documentation** as needed
5. **Ensure all tests pass**
6. **Write clear commit messages**

## Development Setup

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake libboost-all-dev

# For ROS2 development
sudo apt install ros-humble-desktop-full
```

### Building

```bash
git clone <your-fork>
cd ddsm115_driver
./build.sh
```

### Running Tests

```bash
cd build
make test  # Run unit tests (when available)

# Manual testing with examples
./examples/basic_example
```

## Coding Standards

### C++ Style Guidelines

- **C++17 standard** - Use modern C++ features appropriately
- **Header guards** - Use `#pragma once` for header files
- **Naming conventions**:
  - Classes: `PascalCase` (e.g., `DDSM115Driver`)
  - Functions/methods: `snake_case` (e.g., `set_velocity`)
  - Variables: `snake_case` with trailing underscore for members (e.g., `motor_id_`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `PACKET_SIZE`)
  - Namespaces: `lowercase` (e.g., `ddsm115`)

- **Code formatting**:
  - 4 spaces for indentation (no tabs)
  - 100 character line limit
  - Braces on same line for control structures
  - Consistent spacing around operators

### Example Code Style

```cpp
namespace ddsm115 {

class MotorController {
public:
    explicit MotorController(uint8_t motor_id);
    
    bool set_velocity(int16_t velocity_rpm, 
                     uint8_t acceleration_time = 1, 
                     bool brake = false);
    
private:
    static constexpr size_t BUFFER_SIZE = 256;
    
    uint8_t motor_id_;
    std::mutex status_mutex_;
};

} // namespace ddsm115
```

### Documentation

- **Doxygen comments** for public API
- **Inline comments** for complex logic
- **README updates** for new features
- **Example code** for significant functionality

```cpp
/**
 * @brief Set motor velocity in RPM
 * @param velocity_rpm Target velocity (-330 to 330 RPM)
 * @param acceleration_time Acceleration time per 1rpm (0.1ms units)
 * @param brake Apply brake flag (velocity mode only)
 * @return true if command sent successfully, false otherwise
 */
bool set_velocity(int16_t velocity_rpm, 
                 uint8_t acceleration_time = 1, 
                 bool brake = false);
```

## Project Structure

```
ddsm115_driver/
├── include/ddsm115_driver/     # Public headers
├── src/                        # Implementation files
├── examples/                   # Usage examples
├── ros2_wrapper/              # ROS2 integration
├── tests/                     # Unit tests (future)
├── docs/                      # Additional documentation
└── CMakeLists.txt             # Build configuration
```

## Testing Guidelines

### Manual Testing

1. **Hardware testing** with actual DDSM115 motors
2. **Error condition testing** (disconnected hardware, etc.)
3. **Multi-motor testing** on shared RS485 bus
4. **Performance testing** with high-frequency commands

### Automated Testing (Future)

- Unit tests for core functionality
- Integration tests with mock hardware
- Performance benchmarks
- Memory leak detection

## Commit Message Format

Use clear, descriptive commit messages:

```
type(scope): short description

Longer description if needed.

- List any breaking changes
- Reference issues: Fixes #123
```

**Types**: feat, fix, docs, style, refactor, test, chore

**Examples**:
- `feat(driver): add position control mode`
- `fix(ros2): resolve callback thread safety issue`
- `docs(readme): update installation instructions`

## Release Process

1. Update version numbers in:
   - `CMakeLists.txt`
   - `package.xml` (ROS2)
   - `CHANGELOG.md`

2. Create release branch: `release/v1.x.x`
3. Final testing and documentation review
4. Create pull request to main
5. Tag release after merge: `git tag v1.x.x`

## Hardware Testing

If you have access to DDSM115 hardware:

1. **Test all control modes** (current, velocity, position)
2. **Verify error handling** with various fault conditions
3. **Test multi-motor scenarios** if possible
4. **Performance testing** with high-frequency commands
5. **Document any hardware-specific findings**

## ROS2 Integration

For ROS2 contributions:

1. Follow **ROS2 coding standards**
2. Use **standard ROS2 message types** when possible
3. **Parameter validation** for configuration
4. **Namespace support** for multi-robot systems
5. **Launch file examples** for common use cases

## Questions and Support

- **GitHub Issues** - For bug reports and feature requests
- **GitHub Discussions** - For questions and community support
- **Pull Request Reviews** - For code-specific discussions

## Recognition

Contributors will be acknowledged in:
- README.md contributors section
- Release notes
- CHANGELOG.md

Thank you for helping make the DDSM115 Driver Library better!

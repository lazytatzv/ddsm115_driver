# GitHub Repository Setup Guide

This guide helps you set up and deploy the DDSM115 Driver Library to GitHub.

## Quick Setup (Automated)

1. **Run the deployment script:**
   ```bash
   ./deploy.sh
   ```
   
   The script will:
   - Build and test the library
   - Initialize git repository (if needed)
   - Guide you through GitHub setup
   - Create version tags
   - Push to GitHub

## Manual Setup

### 1. Create GitHub Repository

1. Go to [GitHub.com](https://github.com) and sign in
2. Click "New" repository or go to [github.com/new](https://github.com/new)
3. Repository settings:
   - **Name**: `ddsm115_driver` (or your preferred name)
   - **Description**: `C++ library for controlling DDSM115 servo motors via RS485`
   - **Visibility**: Public (recommended) or Private
   - **Initialize**: Don't initialize with README (we have our own)
4. Click "Create repository"

### 2. Configure Repository Topics

Add these topics to help others discover your repository:
- `ddsm115`
- `servo-motor`
- `rs485`
- `cpp`
- `robotics`
- `motor-control`
- `ros2`
- `boost-asio`

### 3. Local Git Setup

```bash
# Initialize git repository (if not already done)
cd ddsm115_driver
git init

# Add all files
git add .

# Initial commit
git commit -m "Initial commit: DDSM115 Driver Library v1.0.0"

# Add GitHub remote (replace with your repository URL)
git remote add origin https://github.com/your-username/ddsm115_driver.git

# Push to GitHub
git branch -M main
git push -u origin main
```

### 4. Create First Release

```bash
# Create and push version tag
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0
```

### 5. Repository Configuration

#### Branch Protection (Recommended)
1. Go to Settings → Branches
2. Add rule for `main` branch:
   - Require pull request reviews
   - Require status checks (CI/CD)
   - Restrict pushes to main

#### Repository Settings
1. **About section**: Add description and website
2. **Topics**: Add relevant tags
3. **Releases**: GitHub Actions will create these automatically
4. **Packages**: Docker images will be published here

## Repository Structure

After setup, your repository will have:

```
ddsm115_driver/
├── .github/
│   ├── ISSUE_TEMPLATE/          # Bug report and feature request templates
│   └── workflows/               # CI/CD workflows
├── include/ddsm115_driver/      # Public headers
├── src/                         # Implementation
├── examples/                    # Usage examples
├── ros2_wrapper/               # ROS2 integration
├── docs/                       # Documentation (optional)
├── README.md                   # Main documentation
├── LICENSE                     # MIT license
├── CHANGELOG.md               # Version history
├── CONTRIBUTING.md            # Contribution guidelines
├── .gitignore                 # Git ignore rules
├── CMakeLists.txt             # Build configuration
├── Dockerfile                 # Docker support
├── build.sh                   # Build script
└── deploy.sh                  # Deployment script
```

## GitHub Features

### Automatic CI/CD
- **Build testing** on Ubuntu 20.04, 22.04, 24.04
- **Compiler testing** with GCC and Clang
- **ROS2 integration** testing with Humble and Iron
- **Code quality** checks with cppcheck and clang-format
- **Documentation** validation

### Automatic Releases
When you push a version tag (e.g., `v1.0.0`):
- **Source packages** are created automatically
- **Binary packages** for Linux x64
- **ROS2 packages** separately packaged
- **Docker images** published to GitHub Container Registry
- **Release notes** extracted from CHANGELOG.md

### Issue Templates
- **Bug reports** with environment details
- **Feature requests** with use case analysis
- **Structured information** for better support

## Publishing to Package Managers

### Conan (C++ Package Manager)
```bash
# Create conanfile.py for your library
# Submit to ConanCenter
```

### ROS2 Package Index
```bash
# Submit to rosdistro for official ROS2 distribution
# Follow ROS2 package guidelines
```

### vcpkg (Microsoft Package Manager)
```bash
# Create vcpkg port
# Submit pull request to vcpkg repository
```

## Best Practices

### Version Management
- Use **semantic versioning** (MAJOR.MINOR.PATCH)
- Update **CHANGELOG.md** before each release
- Create **release notes** with key features and fixes
- **Tag releases** consistently

### Documentation
- Keep **README.md** up to date
- Provide **clear examples**
- Document **API changes** in releases
- Include **hardware setup** guides

### Community
- Respond to **issues** promptly
- Review **pull requests** thoroughly
- Welcome **contributions** from community
- Maintain **code quality** standards

## Troubleshooting

### Permission Issues
```bash
# If you get permission errors
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# For SSH key issues
ssh-keygen -t ed25519 -C "your.email@example.com"
# Add the public key to GitHub
```

### Large File Issues
```bash
# If you have large files, use Git LFS
git lfs track "*.so"
git lfs track "*.a"
git add .gitattributes
```

### CI/CD Failures
- Check the **Actions** tab for detailed logs
- Ensure all **dependencies** are properly declared
- Verify **CMake configuration** works on clean systems
- Test **Docker builds** locally

## Support

After deployment, you can provide support through:
- **GitHub Issues** for bug reports
- **GitHub Discussions** for questions
- **Pull Requests** for contributions
- **Wiki** for additional documentation

Your library is now ready for the open-source community! 🚀

#!/bin/bash

# GitHub Deployment Script for DDSM115 Driver Library
# This script helps prepare and deploy the library to GitHub

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}DDSM115 Driver Library - GitHub Deployment Script${NC}"
echo "=================================================="

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command_exists git; then
    echo -e "${RED}Error: git is not installed${NC}"
    exit 1
fi

if ! command_exists cmake; then
    echo -e "${RED}Error: cmake is not installed${NC}"
    exit 1
fi

if ! command_exists g++; then
    echo -e "${RED}Error: g++ is not installed${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All prerequisites met${NC}"

# Clean previous builds
echo -e "${YELLOW}Cleaning previous builds...${NC}"
rm -rf build/
rm -f test_functionality test_functionality_static

# Build and test the library
echo -e "${YELLOW}Building and testing library...${NC}"
mkdir build
cd build

# Set compilers explicitly to avoid ccache issues
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX
make -j$(nproc)
cd ..

# Run functionality test
echo -e "${YELLOW}Running functionality test...${NC}"
g++ -std=c++17 -I include/ test_functionality.cpp src/ddsm115_driver.cpp -lboost_system -pthread -o test_functionality_static
./test_functionality_static

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Functionality test passed${NC}"
else
    echo -e "${RED}✗ Functionality test failed${NC}"
    exit 1
fi

# Check git repository status
echo -e "${YELLOW}Checking git repository...${NC}"

if [ ! -d ".git" ]; then
    echo -e "${YELLOW}Initializing git repository...${NC}"
    git init
    git add .
    git commit -m "Initial commit: DDSM115 Driver Library v1.0.0"
else
    echo -e "${GREEN}✓ Git repository exists${NC}"
    
    # Check for uncommitted changes
    if ! git diff-index --quiet HEAD --; then
        echo -e "${YELLOW}Uncommitted changes detected. Please review:${NC}"
        git status --porcelain
        echo ""
        read -p "Do you want to commit these changes? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            git add .
            echo "Enter commit message: "
            read commit_message
            git commit -m "$commit_message"
        else
            echo -e "${YELLOW}Please commit or stash changes before deployment${NC}"
            exit 1
        fi
    fi
fi

# Check for remote repository
echo -e "${YELLOW}Checking GitHub repository setup...${NC}"

if ! git remote get-url origin >/dev/null 2>&1; then
    echo -e "${YELLOW}No remote origin found.${NC}"
    echo "Please set up your GitHub repository first:"
    echo ""
    echo "1. Create a new repository on GitHub"
    echo "2. Copy the repository URL"
    echo "3. Run: git remote add origin <repository-url>"
    echo ""
    read -p "Enter your GitHub repository URL: " repo_url
    
    if [ -n "$repo_url" ]; then
        git remote add origin "$repo_url"
        echo -e "${GREEN}✓ Remote origin added${NC}"
    else
        echo -e "${RED}No URL provided. Please set up remote manually.${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ Remote origin configured${NC}"
    git remote get-url origin
fi

# Version tagging
echo -e "${YELLOW}Version tagging...${NC}"
echo "Current tags:"
git tag -l | tail -5

echo ""
read -p "Enter version for this release (e.g., v1.0.0): " version

if [ -n "$version" ]; then
    # Validate version format
    if [[ $version =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo -e "${GREEN}✓ Valid version format: $version${NC}"
        
        # Check if tag already exists
        if git rev-parse "$version" >/dev/null 2>&1; then
            echo -e "${RED}Error: Tag $version already exists${NC}"
            exit 1
        fi
        
        # Create and push tag
        git tag -a "$version" -m "Release $version"
        echo -e "${GREEN}✓ Tag $version created${NC}"
    else
        echo -e "${RED}Error: Invalid version format. Use format: v1.0.0${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}Skipping version tagging${NC}"
fi

# Final deployment confirmation
echo ""
echo -e "${BLUE}Ready for deployment!${NC}"
echo "The following will be pushed to GitHub:"
echo "- All committed changes"
if [ -n "$version" ]; then
    echo "- Version tag: $version"
fi
echo ""

read -p "Proceed with GitHub deployment? (y/N): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Pushing to GitHub...${NC}"
    
    # Push main branch
    git push origin main || git push origin master || {
        echo -e "${YELLOW}Attempting to push current branch...${NC}"
        git push origin HEAD
    }
    
    # Push tags if created
    if [ -n "$version" ]; then
        git push origin "$version"
    fi
    
    echo -e "${GREEN}🎉 Successfully deployed to GitHub!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Visit your GitHub repository to verify the upload"
    echo "2. Create a release from the tag (if tagged)"
    echo "3. Update the repository description and topics"
    echo "4. Add repository URL to your documentation"
    echo ""
    echo "GitHub Actions will automatically:"
    echo "- Build and test the library"
    echo "- Run code quality checks"
    echo "- Create release packages (if tagged)"
    
else
    echo -e "${YELLOW}Deployment cancelled${NC}"
    exit 0
fi

echo ""
echo -e "${GREEN}Deployment completed successfully!${NC}"

# Humanoid Robotics with AI

A comprehensive educational book on building intelligent humanoid robots, delivered as an interactive Docusaurus website.

## 📚 Overview

This project provides in-depth learning materials for robotics developers interested in humanoid robotics with AI integration. The content is structured as modules covering different aspects of the robotics stack.

### Current Modules

**Module 1: The Robotic Nervous System (ROS 2)** - Complete ✅
- Chapter 1: ROS 2 Architecture & Philosophy
- Chapter 2: Communication Primitives (Planned)
- Chapter 3: Python Implementation with rclpy (Planned)
- Chapter 4: URDF for Humanoid Robots (Planned)

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18.0 or higher
- **npm** or **yarn**

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd RoboticAI_book2

# Install dependencies
npm install

# Start development server
npm start
```

The site will open at `http://localhost:3000/`

### Building for Production

```bash
# Create optimized production build
npm run build

# Serve the build locally
npm run serve
```

## 📖 Content Structure

```
docs/
├── module-1-ros2/          # Module 1: ROS 2
│   ├── index.mdx           # Module landing page
│   ├── architecture/       # Chapter 1: Architecture
│   ├── communication/      # Chapter 2: Communication
│   ├── python-rclpy/       # Chapter 3: Python Implementation
│   └── urdf/               # Chapter 4: URDF
│
static/
├── img/
│   └── module-1/           # Diagrams and images
│
examples/
└── module-1/               # ROS 2 code examples
```

## 🛠️ Development

### Running Locally

```bash
npm start
```

This starts a local development server with hot reload.

### Creating Content

1. Create `.mdx` files in the appropriate `docs/` subdirectory
2. Add front matter (title, sidebar_position, description, keywords)
3. Use Docusaurus components (admonitions, tabs, code blocks)
4. Save - the site will hot-reload automatically

### Adding Diagrams

Place SVG diagrams in `static/img/module-X/` and reference them in MDX:

```mdx
![Diagram Description](/img/module-1/diagram-name.svg)
```

### Code Examples

Place runnable code examples in `examples/module-X/` as ROS 2 packages.

## 📋 Prerequisites for Learners

To work through this book, readers should have:

- **Python 3 basics**: Variables, functions, classes
- **Linux command line**: Terminal navigation, running commands
- **Ubuntu 22.04** with **ROS 2 Humble** installed

Installation instructions are provided in Module 1, Chapter 1.

## 🎯 Learning Objectives

### Module 1: The Robotic Nervous System (ROS 2)

- Understand middleware and ROS 2 architecture
- Design robot communication patterns
- Implement Python robot agents with rclpy
- Define humanoid robot structures with URDF

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

### Content Guidelines

- Keep explanations clear and beginner-friendly
- Include code examples that are tested and working
- Add diagrams to illustrate complex concepts
- Provide troubleshooting guidance
- Follow the existing content structure

## 📄 License

[Add your license here]

## 🔗 Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [Project Repository](https://github.com/your-org/humanoid-robotics-with-ai)

## 📧 Contact

[Add contact information here]

---

**Built with** [Docusaurus](https://docusaurus.io/) • **Powered by** [ROS 2 Humble](https://docs.ros.org/en/humble/)

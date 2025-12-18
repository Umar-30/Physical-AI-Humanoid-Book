# Docusaurus Structure Research Summary for Technical Robotics Book

**Project**: Physical AI & Humanoid Robotics Technical Book
**Platform**: Docusaurus 3.0
**Context**: Module 2 (4 chapters) - Building on existing Module 1 structure

---

## 1. Recommended Directory Structure for Multi-Module Book

Based on analysis of your existing Module 1 structure, here's the recommended organization:

```
docs/
├── intro.md                          # Landing page (optional)
├── module-1-ros2/                    # Module 1: The Robotic Nervous System
│   ├── index.mdx                     # Module overview page
│   ├── architecture/
│   │   ├── index.mdx                 # Chapter 1 overview
│   │   ├── middleware-role.mdx
│   │   ├── ros2-overview.mdx
│   │   ├── ros1-vs-ros2.mdx
│   │   ├── dds-explained.mdx
│   │   └── getting-started.mdx
│   ├── communication/
│   │   ├── index.mdx                 # Chapter 2 overview
│   │   ├── topics-pubsub.mdx
│   │   ├── services-reqrep.mdx
│   │   ├── actions-goals.mdx
│   │   ├── qos-profiles.mdx
│   │   ├── executors.mdx
│   │   └── graph-design.mdx
│   ├── python-rclpy/
│   │   ├── index.mdx                 # Chapter 3 overview
│   │   ├── rclpy-overview.mdx
│   │   ├── package-setup.mdx
│   │   ├── publishers-subscribers.mdx
│   │   ├── services.mdx
│   │   ├── actions.mdx
│   │   └── custom-messages.mdx
│   └── urdf/
│       ├── index.mdx                 # Chapter 4 overview
│       ├── urdf-basics.mdx
│       ├── links-joints.mdx
│       ├── coordinate-frames.mdx
│       ├── humanoid-urdf.mdx
│       └── visualization.mdx
│
├── module-2-<name>/                  # Module 2 (NEW)
│   ├── index.mdx                     # Module overview
│   ├── chapter-1/
│   │   ├── index.mdx
│   │   ├── section-1.mdx
│   │   ├── section-2.mdx
│   │   └── ...
│   ├── chapter-2/
│   ├── chapter-3/
│   └── chapter-4/
│
└── module-N/                         # Future modules

static/
├── img/
│   ├── favicon.ico
│   ├── logo.svg
│   ├── module-1/                     # Module-specific images
│   │   ├── ros2-architecture.png
│   │   ├── dds-layers.svg
│   │   └── ...
│   ├── module-2/                     # Module 2 images
│   │   └── ...
│   └── diagrams/                     # Shared diagrams
│       └── ...
├── files/                            # Downloadable files
│   ├── module-1/
│   │   ├── config/
│   │   │   ├── example-qos.yaml
│   │   │   └── package.xml
│   │   └── code/
│   │       ├── talker.py
│   │       └── listener.py
│   └── module-2/
│       └── ...
└── downloads/                        # PDFs, slides, etc.
    ├── module-1-slides.pdf
    └── ...
```

### Key Organizational Principles

1. **Module-Based Top Level**: Each module gets its own directory (`module-1-ros2`, `module-2-vision`, etc.)
2. **Chapter Subdirectories**: Within each module, chapters are organized as subdirectories
3. **Index Pages**: Every module and chapter has an `index.mdx` serving as an overview/landing page
4. **Flat Section Structure**: Individual sections are files within the chapter directory (not nested further)
5. **Asset Co-location**: Static assets organized by module for easy management

---

## 2. Front Matter Template for Chapter Pages

### Module Index Page Template

```yaml
---
sidebar_position: 1
title: Module N - Module Title
description: Brief module description (1-2 sentences)
keywords: [keyword1, keyword2, keyword3, robotics, ai]
slug: /module-n-slug
---
```

**Example** (from existing Module 1):
```yaml
---
sidebar_position: 1
title: Module 1 - The Robotic Nervous System (ROS 2)
description: Learn the foundational middleware layer for humanoid robotics using ROS 2
keywords: [ros2, robotics, middleware, humanoid, rclpy, urdf]
---
```

### Chapter Index Page Template

```yaml
---
sidebar_position: N
title: Chapter N - Chapter Title
description: Brief chapter description
keywords: [chapter-specific, keywords, related, terms]
---
```

**Example** (from Chapter 1):
```yaml
---
sidebar_position: 1
title: Chapter 1 - ROS 2 Architecture & Philosophy
description: Understanding the fundamental role of middleware in robotics systems
keywords: [ros2, architecture, middleware, dds, nodes]
---
```

### Section/Topic Page Template

```yaml
---
sidebar_position: N
title: Section Title
description: Brief section description (used in SEO and link previews)
keywords: [specific, technical, terms, for, seo]
---
```

**Example** (from Getting Started):
```yaml
---
sidebar_position: 6
title: Getting Started with ROS 2
description: Install ROS 2 Humble and run your first nodes
keywords: [ros2, installation, humble, ubuntu, commands, tutorial]
---
```

### Advanced Front Matter Options

For specialized pages, you can add:

```yaml
---
# Core metadata
sidebar_position: 1
title: Page Title
description: Page description

# SEO
keywords: [keyword1, keyword2]
image: /img/social-card.png          # Social media preview image

# Navigation
sidebar_label: "Short Label"          # Override sidebar text
pagination_label: "Custom Label"      # Override pagination text
hide_title: false                     # Hide the title on the page
hide_table_of_contents: false         # Hide TOC sidebar

# Custom
tags: [tutorial, advanced, python]    # If using doc tags plugin
custom_edit_url: https://...          # Override edit URL
---
```

### Front Matter Best Practices

1. **Always include**: `sidebar_position`, `title`, `description`
2. **Keywords**: 3-7 relevant technical terms for SEO
3. **Descriptions**: Keep to 1-2 sentences, focus on learning outcomes or value
4. **Sidebar Position**: Number sequentially (1, 2, 3...) within each category
5. **Title Format**: Use consistent patterns
   - Module: "Module N - Title"
   - Chapter: "Chapter N - Title" or "Chapter N: Title"
   - Section: Plain descriptive title

---

## 3. Code Block Best Practices

### Basic Syntax Highlighting

Docusaurus uses Prism for syntax highlighting. Your config already includes:

```javascript
prism: {
  theme: lightCodeTheme,
  darkTheme: darkCodeTheme,
  additionalLanguages: ['bash', 'python'],
}
```

#### Supported by Default
- JavaScript, TypeScript, JSX, TSX
- CSS, HTML
- JSON, YAML
- Markdown

#### Currently Added
- `bash` (shell commands)
- `python`

#### Additional Languages for Robotics Book

Add to `additionalLanguages` array in `docusaurus.config.js`:

```javascript
additionalLanguages: [
  'bash',
  'python',
  'cpp',           // C++ code
  'cmake',         // CMake files
  'xml',           // URDF, launch files, package.xml
  'yaml',          // Config files, YAML launch files
  'docker',        // Dockerfiles
  'diff',          // Code diffs
  'ini',           // Configuration files
  'makefile',      // Makefiles
]
```

### Code Block Syntax

#### Basic Code Block

````markdown
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Publisher node started')
```
````

#### Code Block with Title

````markdown
```python title="src/my_package/talker.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
```
````

#### Highlight Specific Lines

````markdown
```python {3,5-7}
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Highlighted

class TalkerNode(Node):           # Highlighted
    def __init__(self):            # Highlighted
        super().__init__('talker') # Highlighted
```
````

#### Line Numbers

````markdown
```python showLineNumbers
def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
````

#### Combined: Title + Line Numbers + Highlighting

````markdown
```python title="talker.py" showLineNumbers {4-6}
import rclpy
from rclpy.node import Node

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
```
````

### Command-Line Examples

#### Single Command

````markdown
```bash
ros2 run demo_nodes_cpp talker
```
````

#### Command with Output

````markdown
```bash
ros2 node list
```

**Output**:
```
/talker
/listener
```
````

Or using comments:

````markdown
```bash
ros2 node list
# Output:
# /talker
# /listener
```
````

#### Multi-Line Commands

````markdown
```bash
# Update apt repository caches
sudo apt update

# Install ROS 2 Humble Desktop
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install python3-colcon-common-extensions
```
````

#### Interactive Command Sessions

````markdown
```bash
$ ros2 topic echo /chatter
data: 'Hello World: 1'
---
data: 'Hello World: 2'
---
# Press Ctrl+C to stop
```
````

### Configuration File Examples

#### YAML Configuration

````markdown
```yaml title="config/qos_profile.yaml"
publisher:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: transient_local
```
````

#### XML (URDF, Launch Files)

````markdown
```xml title="robot.urdf"
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```
````

#### Package.xml

````markdown
```xml title="package.xml"
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>My robot package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
</package>
```
````

### Code Tabs for Multiple Languages/Approaches

Install the code tabs plugin first:

```bash
npm install @docusaurus/theme-code-tabs
```

Then use:

````markdown
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs groupId="programming-language">
  <TabItem value="python" label="Python" default>

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```

  </TabItem>
  <TabItem value="cpp" label="C++">

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {}
};
```

  </TabItem>
</Tabs>
````

### Code Block Best Practices Summary

1. **Always specify language** for syntax highlighting
2. **Use titles** for file paths to provide context
3. **Show line numbers** for longer code blocks (>10 lines)
4. **Highlight key lines** to draw attention to important changes
5. **Separate commands from output** using comments or separate blocks
6. **Use tabs** when showing equivalent code in multiple languages
7. **Keep examples concise** - prefer complete minimal examples over snippets

---

## 4. Admonitions for Educational Content

### Available Admonition Types

Docusaurus provides 5 built-in admonition types:

1. `note` - General information
2. `tip` - Helpful suggestions
3. `info` - Important information
4. `warning` - Warnings about potential issues
5. `danger` - Critical warnings

### Basic Syntax

```markdown
:::note
This is a note
:::

:::tip
This is a helpful tip
:::

:::info
This is important information
:::

:::warning
This is a warning
:::

:::danger
This is a critical warning
:::
```

### With Custom Titles

```markdown
:::note Custom Title
Content goes here
:::

:::tip Pro Tip
Use custom titles to make admonitions more specific
:::
```

### Multi-Line Content

```markdown
:::info ROS 2 Distributions

ROS 2 has several distributions:
- **Humble Hawksbill** (2022-2027) - LTS
- **Iron Irwini** (2023-2024)
- **Jazzy Jalisco** (2024-2026)

Always use an LTS release for production.
:::
```

### Nested Content (Code Blocks, Lists)

```markdown
:::tip Quick Setup Check

Verify your ROS 2 installation:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

Expected output: `ros2 run humble`

:::
```

### Educational Content Patterns

#### 1. Prerequisites Check

```markdown
:::note System Requirements

Before proceeding, ensure you have:
- Ubuntu 22.04 LTS (64-bit)
- At least 5 GB free disk space
- Active internet connection

:::
```

#### 2. Learning Tips

```markdown
:::tip Best Practice

Always source your workspace setup script before running ROS 2 commands:

```bash
source /opt/ros/humble/setup.bash
```

Add this to your `~/.bashrc` for automatic sourcing.
:::
```

#### 3. Warnings About Common Mistakes

```markdown
:::warning Common Mistake

Don't forget to call `rclpy.init()` before creating nodes:

```python
rclpy.init(args=args)  # Required!
node = MyNode()
```

Otherwise you'll get a runtime error.
:::
```

#### 4. Critical Safety Information

```markdown
:::danger Safety Critical

Never bypass safety limits in humanoid robot control code. Always implement:
- Joint limit checks
- Emergency stop handlers
- Watchdog timers

Failure to do so can result in hardware damage or injury.
:::
```

#### 5. Explanatory Info Boxes

```markdown
:::info Why Ubuntu 22.04?

Ubuntu 22.04 LTS is the officially supported platform for ROS 2 Humble. While ROS 2 can run on Windows and macOS, Ubuntu provides the best experience for learning.
:::
```

#### 6. Key Takeaways / Summary

```markdown
:::note Summary

- **ROS 2 Humble** is installed via apt on Ubuntu 22.04
- **Source setup script** in every terminal: `source /opt/ros/humble/setup.bash`
- **Talker/Listener demo** shows automatic node discovery
- **CLI tools**: `ros2 node`, `ros2 topic`, `ros2 interface`

:::
```

#### 7. Advanced Topic Callouts

```markdown
:::info Advanced: Custom QoS Profiles

For advanced use cases, you can define custom Quality of Service profiles:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)
```

We'll cover this in Chapter 2.
:::
```

### Admonition Usage Guidelines

**Use `note` for:**
- General information
- Context that helps understanding
- Background information
- Summaries

**Use `tip` for:**
- Best practices
- Helpful shortcuts
- Pro tips
- Optimization suggestions
- Productivity enhancements

**Use `info` for:**
- Important clarifications
- "Why" explanations
- Version-specific information
- Platform-specific notes

**Use `warning` for:**
- Common mistakes to avoid
- Potential pitfalls
- Performance considerations
- Deprecation notices

**Use `danger` for:**
- Safety-critical information
- Data loss warnings
- Security concerns
- Breaking changes

### Frequency Recommendations

- **Don't overuse**: 2-3 admonitions per page maximum (for average-length pages)
- **Strategic placement**: Use at decision points, before complex sections, or after examples
- **Vary types**: Mix types to avoid monotony
- **Keep concise**: 2-4 sentences or bullet points maximum

---

## 5. Asset Management Strategy

### Image Organization

#### Directory Structure

```
static/
├── img/
│   ├── favicon.ico
│   ├── logo.svg
│   ├── module-1/
│   │   ├── architecture/
│   │   │   ├── ros2-layers.png
│   │   │   ├── dds-architecture.svg
│   │   │   └── node-discovery.png
│   │   ├── communication/
│   │   │   ├── topic-pattern.svg
│   │   │   ├── service-pattern.svg
│   │   │   └── action-pattern.svg
│   │   └── urdf/
│   │       ├── humanoid-model.png
│   │       └── joint-hierarchy.svg
│   ├── module-2/
│   │   └── ...
│   └── shared/
│       ├── robot-photo.jpg
│       └── ai-concept.svg
```

#### Embedding Images in MDX

**Standard Markdown:**
```markdown
![ROS 2 Architecture Diagram](/img/module-1/architecture/ros2-layers.png)
```

**With Alt Text (Accessibility):**
```markdown
![Diagram showing the layered architecture of ROS 2, from application layer down to DDS middleware](/img/module-1/architecture/ros2-layers.png)
```

**Centered with Title:**
```markdown
<div style={{textAlign: 'center'}}>
  <img
    src="/img/module-1/architecture/ros2-layers.png"
    alt="ROS 2 Architecture Layers"
    style={{maxWidth: '600px'}}
  />
  <p><em>Figure 1.1: ROS 2 Layered Architecture</em></p>
</div>
```

**Using MDX for Responsive Images:**
```jsx
import useBaseUrl from '@docusaurus/useBaseUrl';

<img
  src={useBaseUrl('/img/module-1/urdf/humanoid-model.png')}
  alt="Humanoid robot URDF visualization"
  style={{maxWidth: '100%', height: 'auto'}}
/>
```

### Downloadable Files

#### Configuration Files

```
static/
└── files/
    └── module-1/
        ├── config/
        │   ├── qos_profile.yaml
        │   ├── robot_params.yaml
        │   └── launch_config.py
        ├── urdf/
        │   ├── humanoid_basic.urdf
        │   └── humanoid_full.urdf
        └── code/
            ├── talker.py
            ├── listener.py
            └── service_example.py
```

#### Linking to Downloads

**Direct Download Link:**
```markdown
Download the complete URDF file: [humanoid_full.urdf](/files/module-1/urdf/humanoid_full.urdf)
```

**Code File with Explanation:**
```markdown
Here's the complete implementation:

[📥 Download talker.py](/files/module-1/code/talker.py)

```python title="talker.py"
# Code preview shown here...
```

You can run this file directly:
```bash
ros2 run my_package talker
```
```

**Configuration Download:**
```markdown
:::tip Download Configuration

Download the QoS configuration file: [qos_profile.yaml](/files/module-1/config/qos_profile.yaml)

Place it in your package's `config/` directory.
:::
```

### Diagrams and SVGs

#### When to Use SVG vs PNG

**Use SVG for:**
- Architecture diagrams
- Flow charts
- Network diagrams
- Simple illustrations
- Graphs and charts

**Advantages:**
- Scales perfectly at any size
- Smaller file size
- Can be edited easily
- Better for light/dark themes

**Use PNG/JPG for:**
- Screenshots
- Photos
- Complex renders
- When SVG editing tools aren't available

#### Inline SVG in MDX (for simple diagrams)

```jsx
<svg width="300" height="100" xmlns="http://www.w3.org/2000/svg">
  <rect x="10" y="10" width="80" height="80" fill="lightblue" />
  <text x="50" y="55" text-anchor="middle">Node A</text>
</svg>
```

#### Theme-Aware Images

For diagrams that need different versions for light/dark themes:

```jsx
import useBaseUrl from '@docusaurus/useBaseUrl';
import ThemedImage from '@theme/ThemedImage';

<ThemedImage
  alt="ROS 2 Architecture"
  sources={{
    light: useBaseUrl('/img/module-1/architecture-light.svg'),
    dark: useBaseUrl('/img/module-1/architecture-dark.svg'),
  }}
/>
```

### Asset Optimization

#### Image Size Guidelines

- **Diagrams/SVG**: Keep source clean, optimize with SVGO
- **Screenshots**:
  - Max width: 1920px
  - Use PNG for UI screenshots
  - Compress with tools like TinyPNG
- **Photos**:
  - Max width: 1200px
  - Use JPG, quality 80-85%
- **Icons/Logos**:
  - Use SVG when possible
  - If PNG, provide 2x retina versions

#### File Naming Conventions

```
kebab-case-naming.png
ros2-architecture-diagram.svg
service-pattern-example.png
module-1-chapter-2-fig-1.png
```

Best practices:
- Use descriptive names
- Include context (module/chapter)
- Use kebab-case
- Avoid spaces
- Include version numbers if applicable (e.g., `diagram-v2.svg`)

### Asset Best Practices Summary

1. **Organize by module** to keep assets manageable
2. **Use SVG for diagrams** for scalability and editing
3. **Provide alt text** for all images (accessibility)
4. **Optimize file sizes** before committing
5. **Use meaningful names** that describe content
6. **Version control friendly**: Prefer text-based formats (SVG) when possible
7. **Download links**: Provide complete, runnable examples
8. **Consistent styling**: Use similar visual styles across diagrams

---

## 6. Navigation and Sidebar Configuration

### Current Sidebar Configuration (sidebars.js)

Your existing structure uses:

```javascript
const sidebars = {
  moduleSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture',
          link: {
            type: 'doc',
            id: 'module-1-ros2/architecture/index',
          },
          items: [
            'module-1-ros2/architecture/middleware-role',
            'module-1-ros2/architecture/ros2-overview',
            // ... more sections
          ],
        },
        // ... more chapters
      ],
    },
  ],
};
```

### Extending for Module 2

#### Option 1: Single Sidebar with Multiple Modules

```javascript
const sidebars = {
  bookSidebar: [
    // Module 1
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      link: { type: 'doc', id: 'module-1-ros2/index' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Architecture',
          link: { type: 'doc', id: 'module-1-ros2/architecture/index' },
          items: [
            'module-1-ros2/architecture/middleware-role',
            'module-1-ros2/architecture/ros2-overview',
            'module-1-ros2/architecture/ros1-vs-ros2',
            'module-1-ros2/architecture/dds-explained',
            'module-1-ros2/architecture/getting-started',
          ],
        },
        // ... more Module 1 chapters
      ],
    },

    // Module 2
    {
      type: 'category',
      label: 'Module 2: Computer Vision',
      link: { type: 'doc', id: 'module-2-vision/index' },
      collapsed: true,  // Start collapsed
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Image Processing',
          link: { type: 'doc', id: 'module-2-vision/image-processing/index' },
          items: [
            'module-2-vision/image-processing/fundamentals',
            'module-2-vision/image-processing/opencv-basics',
            // ... more sections
          ],
        },
        // ... more Module 2 chapters
      ],
    },
  ],
};
```

#### Option 2: Separate Sidebars Per Module (Recommended for Large Books)

```javascript
const sidebars = {
  // Module 1 Sidebar
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: { type: 'doc', id: 'module-1-ros2/index' },
      items: [
        // ... chapters
      ],
    },
  ],

  // Module 2 Sidebar
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: Vision',
      link: { type: 'doc', id: 'module-2-vision/index' },
      items: [
        // ... chapters
      ],
    },
  ],
};
```

Then update `docusaurus.config.js` navbar:

```javascript
navbar: {
  items: [
    {
      type: 'doc',
      docId: 'module-1-ros2/index',
      position: 'left',
      label: 'Module 1: ROS 2',
      docsPluginId: 'default',
      sidebarId: 'module1Sidebar',
    },
    {
      type: 'doc',
      docId: 'module-2-vision/index',
      position: 'left',
      label: 'Module 2: Vision',
      docsPluginId: 'default',
      sidebarId: 'module2Sidebar',
    },
  ],
},
```

### Sidebar Features

#### Collapsible Categories

```javascript
{
  type: 'category',
  label: 'Advanced Topics',
  collapsed: true,  // Start collapsed
  collapsible: true,  // Can be toggled (default: true)
  items: ['advanced/topic1', 'advanced/topic2'],
}
```

#### Auto-Generated Sidebars

For simpler structures, use auto-generation:

```javascript
{
  type: 'autogenerated',
  dirName: 'module-2-vision',
}
```

This automatically generates sidebar from directory structure and front matter.

#### Category Index Pages

The `link` field makes a category clickable:

```javascript
{
  type: 'category',
  label: 'Chapter 1: Architecture',
  link: {
    type: 'doc',
    id: 'module-1-ros2/architecture/index',  // Landing page
  },
  items: [/* sections */],
}
```

Or generated description page:

```javascript
{
  type: 'category',
  label: 'Chapter 1',
  link: {
    type: 'generated-index',
    title: 'Chapter 1 Overview',
    description: 'Learn about ROS 2 architecture',
    slug: '/module-1/chapter-1',
  },
  items: [/* sections */],
}
```

#### Custom Sidebar Item Labels

Override doc title in sidebar:

```javascript
{
  type: 'doc',
  id: 'module-1-ros2/architecture/getting-started',
  label: 'Quick Start',  // Different from page title
}
```

Or use `sidebar_label` in front matter:

```yaml
---
title: Getting Started with ROS 2 Humble Installation
sidebar_label: Quick Start
---
```

### Navigation Best Practices

1. **Depth**: Keep sidebar depth to 3 levels max (Module > Chapter > Section)
2. **Collapsed**: Start with later modules/chapters collapsed
3. **Consistent Labels**: Use consistent naming patterns
4. **Index Pages**: Always provide overview pages for categories
5. **Breadcrumbs**: Docusaurus provides these automatically
6. **Previous/Next**: Auto-generated from sidebar order

### Pagination Control

Control previous/next links in front matter:

```yaml
---
pagination_prev: module-1-ros2/communication/index
pagination_next: module-1-ros2/python-rclpy/index
---
```

Or disable:

```yaml
---
pagination_prev: null
pagination_next: null
---
```

---

## 7. Build Validation Workflow

### Local Development Workflow

#### 1. Start Development Server

```bash
npm start
# or
npm run start
```

**Features:**
- Hot reload on file changes
- Fast incremental builds
- Available at `http://localhost:3000`
- Shows build warnings/errors in real-time

#### 2. Check for Broken Links

Docusaurus detects broken links during build. Your config:

```javascript
onBrokenLinks: 'throw',        // Fail build on broken links
onBrokenMarkdownLinks: 'warn', // Warn on broken markdown links
```

Run production build to validate:

```bash
npm run build
```

**Common issues caught:**
- Broken internal links (`[link](./nonexistent.md)`)
- Missing images
- Invalid doc IDs in sidebar
- Broken cross-references

#### 3. Local Production Build Testing

```bash
# Build for production
npm run build

# Serve the production build locally
npm run serve
```

This tests:
- Production optimizations
- Minification
- Link resolution
- Asset loading

### Pre-Commit Validation

Create a validation script `scripts/validate.sh`:

```bash
#!/bin/bash

echo "Running Docusaurus build validation..."

# Clear previous build
npm run clear

# Run production build
npm run build

# Check exit code
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    exit 0
else
    echo "❌ Build failed! Fix errors before committing."
    exit 1
fi
```

Usage:

```bash
chmod +x scripts/validate.sh
./scripts/validate.sh
```

### GitHub Actions CI Workflow

Create `.github/workflows/build-validation.yml`:

```yaml
name: Build Validation

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Run link checker
        run: |
          npm install -g linkinator
          linkinator ./build --recurse --silent
```

### GitHub Pages Deployment

#### 1. Configure for GitHub Pages

Update `docusaurus.config.js`:

```javascript
module.exports = {
  url: 'https://your-username.github.io',
  baseUrl: '/repo-name/',  // Your repository name
  organizationName: 'your-username',
  projectName: 'repo-name',

  // GitHub Pages deployment config
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
};
```

#### 2. Manual Deployment

```bash
# Set Git user (required for deployment)
GIT_USER=your-username npm run deploy
```

This:
1. Builds the site
2. Pushes to `gh-pages` branch
3. GitHub Pages serves from that branch

#### 3. Automated Deployment with GitHub Actions

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

permissions:
  contents: write

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

#### 4. Enable GitHub Pages

1. Go to repository Settings > Pages
2. Source: Deploy from a branch
3. Branch: `gh-pages` / `root`
4. Save

Site will be available at: `https://your-username.github.io/repo-name/`

### Build Validation Checklist

Pre-deployment checklist:

- [ ] All builds complete without errors
- [ ] No broken internal links
- [ ] All images load correctly
- [ ] Code blocks have proper syntax highlighting
- [ ] Navigation works (sidebar, breadcrumbs, prev/next)
- [ ] Mobile responsive (test at different screen sizes)
- [ ] Search works (if using search plugin)
- [ ] All admonitions render properly
- [ ] Dark mode works (if enabled)

### Common Build Issues

#### Issue: "Error: Duplicate routes found"

**Cause**: Multiple docs with same slug

**Solution**: Ensure unique `slug` in front matter or unique file paths

#### Issue: "Error: Can't resolve './path/to/doc'"

**Cause**: Broken link in markdown or sidebar config

**Solution**: Check file exists, path is correct, and uses forward slashes

#### Issue: "Warning: Broken link on [page]"

**Cause**: Link to non-existent doc

**Solution**: Fix the link or create the missing doc

#### Issue: "Module not found: Can't resolve '@theme/...'"

**Cause**: Missing theme component or incorrect import

**Solution**: Check import path or install missing package

### Performance Optimization

For large documentation sites:

#### 1. Code Splitting

Docusaurus does this automatically, but you can optimize:

```javascript
// docusaurus.config.js
module.exports = {
  future: {
    experimental_faster: true,  // Enable experimental faster builds
  },
};
```

#### 2. Image Optimization

Use webpack plugin:

```bash
npm install @docusaurus/plugin-ideal-image
```

```javascript
// docusaurus.config.js
plugins: [
  '@docusaurus/plugin-ideal-image',
],
```

#### 3. Search Optimization

For large sites, use Algolia DocSearch (free for open source):

```javascript
themeConfig: {
  algolia: {
    appId: 'YOUR_APP_ID',
    apiKey: 'YOUR_API_KEY',
    indexName: 'YOUR_INDEX_NAME',
  },
},
```

---

## 8. Recommended Module 2 Implementation Plan

Based on the research, here's the recommended approach for Module 2:

### Step 1: Create Directory Structure

```
docs/module-2-<name>/
├── index.mdx
├── chapter-1/
│   ├── index.mdx
│   ├── section-1.mdx
│   ├── section-2.mdx
│   └── ...
├── chapter-2/
│   └── ...
├── chapter-3/
│   └── ...
└── chapter-4/
    └── ...
```

### Step 2: Set Up Static Assets

```
static/
├── img/module-2/
│   ├── chapter-1/
│   ├── chapter-2/
│   ├── chapter-3/
│   └── chapter-4/
└── files/module-2/
    ├── config/
    └── code/
```

### Step 3: Update Sidebar Configuration

Add Module 2 to `sidebars.js` following the same pattern as Module 1.

### Step 4: Update Navbar

Add Module 2 to `docusaurus.config.js` navbar items.

### Step 5: Create Content Using Templates

Use the front matter templates and patterns established above.

### Step 6: Validate and Deploy

Follow the build validation workflow before deployment.

---

## 9. Quick Reference: Essential Patterns

### Module Index Page Pattern

```mdx
---
sidebar_position: 1
title: Module N - Title
description: Brief description
keywords: [key, words]
---

# Module N: Title

Introduction paragraph...

## 🎯 Learning Outcomes

1. Outcome 1
2. Outcome 2

## 📚 Module Structure

### Chapter 1: Title
Brief description...

### Chapter 2: Title
Brief description...

## ⏱️ Time Commitment

**Estimated**: X-Y hours

## 📋 Prerequisites

- Prerequisite 1
- Prerequisite 2

## 🚀 Ready to Begin?

Start with [Chapter 1](./chapter-1/)
```

### Chapter Index Page Pattern

```mdx
---
sidebar_position: N
title: Chapter N - Title
description: Brief description
keywords: [keywords]
---

# Chapter N: Title

Introduction...

## 🎯 Learning Objectives

1. Objective 1
2. Objective 2

## 📖 Sections

- [Section 1](./section-1) - Description
- [Section 2](./section-2) - Description

## ⏱️ Estimated Time

**X hours** - Brief note

---

**Next**: [Section 1 →](./section-1)
```

### Section Page Pattern

```mdx
---
sidebar_position: N
title: Section Title
description: Brief description
keywords: [keywords]
---

# Section Title

Introduction paragraph...

## Main Heading

Content with code examples:

```language title="filename"
code here
```

:::tip Best Practice
Helpful tip here
:::

## Summary

Key takeaways...

---

**Next**: [Next Section →](#)
**Previous**: [← Previous Section](#)
```

---

## 10. Additional Resources

### Docusaurus Documentation
- Main Docs: https://docusaurus.io/docs
- MDX Features: https://docusaurus.io/docs/markdown-features
- Configuration: https://docusaurus.io/docs/configuration
- Deployment: https://docusaurus.io/docs/deployment

### Prism Language Support
- Supported Languages: https://prismjs.com/#supported-languages
- Line Highlighting: https://docusaurus.io/docs/markdown-features/code-blocks#line-highlighting

### Accessibility
- Alt Text Guide: https://www.w3.org/WAI/tutorials/images/
- Docusaurus Accessibility: https://docusaurus.io/docs/accessibility

### Diagrams
- Mermaid Plugin: https://docusaurus.io/docs/markdown-features/diagrams (for inline diagrams)
- Draw.io: https://www.drawio.com/ (for complex diagrams exported as SVG)

---

## Summary

This research summary provides:

1. **Directory structure** optimized for multi-module technical books
2. **Front matter templates** for consistent metadata
3. **Code block patterns** for various programming languages and command-line examples
4. **Admonition usage** tailored for educational content
5. **Asset management** strategy for images, diagrams, and downloadable files
6. **Sidebar configuration** for scalable navigation
7. **Build validation workflow** for quality assurance and GitHub Pages deployment

The patterns are based on your existing Module 1 structure and Docusaurus 3.0 best practices, ready to be applied to Module 2 and beyond.

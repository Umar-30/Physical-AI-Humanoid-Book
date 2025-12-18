# Research Document: Module 1 - ROS 2 Educational Content

**Feature**: 001-ros2-module
**Date**: 2025-12-16
**Phase**: Phase 0 - Research & Architecture

## Executive Summary

This document consolidates research findings for creating an educational module on ROS 2 for humanoid robotics. Key decisions include: (1) Using ROS 2 Humble exclusively over ROS 1, (2) Python/rclpy for all code examples, (3) Simplified 8-12 link humanoid URDF, and (4) Hierarchical Docusaurus content structure. All decisions prioritize educational clarity and reproducibility while maintaining technical accuracy.

---

## 1. ROS 2 vs ROS 1 Decision

### Research Question
Should the module teach ROS 1, ROS 2, or both? What justifies focusing exclusively on ROS 2?

### Findings

**ROS 2 Architectural Improvements** (Source: docs.ros.org):
- **DDS Middleware**: Uses Data Distribution Service (OMG standard) for real-time, distributed communication
- **Real-time Support**: Deterministic execution with real-time OS support (not possible in ROS 1)
- **Security**: Built-in DDS security (SROS2) with encryption and authentication
- **Cross-platform**: Native Windows, macOS, Linux support (ROS 1 is Linux-centric)
- **Lifecycle Management**: Managed node states (configuring, active, inactive) for controlled startup/shutdown
- **QoS Policies**: Fine-grained Quality of Service configuration per topic
- **Modern C++17/Python 3**: Current language standards (ROS 1 uses C++03/Python 2)

**ROS 1 vs ROS 2 Comparison**:

| Aspect | ROS 1 Noetic | ROS 2 Humble | Impact for Humanoid Robotics |
|--------|--------------|--------------|------------------------------|
| Real-time | Limited | Full RTOS support | Critical for motor control |
| Security | Basic | DDS-Security | Essential for deployed robots |
| Platform | Linux only | Multi-platform | Broader development options |
| Python | 2.7 (EOL) | 3.10+ | Modern language features |
| Middleware | Custom | DDS (standard) | Industry interoperability |
| EOL | May 2025 | May 2027 | Future-proof content |

**Industry Adoption**:
- Major robotics companies (Boston Dynamics, Clearpath) migrating to ROS 2
- New humanoid platforms (Agility Robotics Digit) use ROS 2
- ROS 1 in maintenance mode, no new features

### Decision

**Use ROS 2 Humble exclusively**

### Rationale

1. **Future-proofing**: ROS 1 Noetic EOL in May 2025, ROS 2 Humble LTS until May 2027
2. **Humanoid Requirements**: Real-time control and security essential for physical robots
3. **Educational Value**: Teaching current standard prepares learners for modern robotics
4. **Ecosystem**: ROS 2 is where new tools, packages, and community focus resides
5. **Simplicity**: Single-framework focus reduces cognitive load for learners

### Alternatives Considered

- **ROS 1 Focus**: Rejected - legacy system, EOL imminent, lacks critical features
- **Dual ROS 1/ROS 2**: Rejected - doubles content complexity, confuses learners, dilutes focus
- **ROS 2 Foxy/Iron**: Rejected - Humble is LTS (longer support), standard for 2024-2025

### References

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Design: https://design.ros2.org/
- DDS Specification: https://www.omg.org/spec/DDS/

---

## 2. rclpy vs rclcpp Choice

### Research Question
Should code examples use Python (rclpy) or C++ (rclcpp)? What best serves beginner-to-intermediate learners?

### Findings

**Python (rclpy) Characteristics**:
- **Syntax**: Simple, readable, minimal boilerplate
- **Learning Curve**: Gentle for those with basic programming knowledge
- **Iteration Speed**: Fast write-test-debug cycles
- **Type Safety**: Dynamic typing (flexible but error-prone)
- **Performance**: Slower than C++, sufficient for many robotics tasks
- **Ecosystem**: Rich scientific libraries (NumPy, SciPy) for data processing

**C++ (rclcpp) Characteristics**:
- **Syntax**: Complex, requires memory management knowledge
- **Learning Curve**: Steep for beginners, compiler errors can be cryptic
- **Iteration Speed**: Slower (compile-link-test cycles)
- **Type Safety**: Static typing (catches errors at compile time)
- **Performance**: Maximum performance, critical for high-rate control loops
- **Ecosystem**: Direct hardware access, real-time capabilities

**Educational Context Analysis**:
- Target audience: Beginner-to-intermediate with Python/Linux basics
- Module focus: Concepts (architecture, communication, modeling), not performance optimization
- Higher-level modules will integrate AI/ML (Python-native domain)
- Official ROS 2 tutorials offer both Python and C++ tracks

**Learning Outcomes Comparison**:

| Learning Goal | Python | C++ | Winner |
|---------------|--------|-----|--------|
| Understand pub/sub concept | ✓✓ | ✓ | Python (less syntax overhead) |
| Grasp QoS policies | ✓✓ | ✓✓ | Tie (both expose QoS clearly) |
| Build first working node | ✓✓ | ✓ | Python (10 lines vs 30+ lines) |
| Debug communication issues | ✓✓ | ✓ | Python (no compile-time confusion) |
| Integrate with AI/ML | ✓✓ | ✗ | Python (native ecosystem) |

### Decision

**Use Python and rclpy exclusively for all code examples**

### Rationale

1. **Accessibility**: Lower barrier to entry, focus on ROS 2 concepts not language syntax
2. **Readability**: Code examples are self-explanatory, reducing explanation overhead
3. **Iteration**: Learners can experiment and see results immediately
4. **Alignment**: Python dominates AI/ML (later modules), establishes consistent language
5. **Community**: Large Python robotics community, abundant resources

### Alternatives Considered

- **C++ (rclcpp)**: Rejected - too steep for target audience, focuses attention on language not concepts
- **Mixed Python/C++**: Rejected - doubles example maintenance, forces syntax comparisons, confuses progression
- **Language-agnostic pseudocode**: Rejected - not executable, violates reproducibility principle

### Performance Note

For production humanoid robots, C++ is often preferred for low-level control loops. However, this module teaches *concepts* and *architecture*. Performance optimization is explicitly out of scope (see spec.md). Learners can apply concepts to C++ later once fundamentals are solid.

### References

- rclpy Documentation: https://docs.ros.org/en/humble/p/rclpy/
- ROS 2 Python Tutorials: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

---

## 3. URDF Complexity Level

### Research Question
What level of URDF complexity balances educational value with comprehensibility? How detailed should the humanoid example be?

### Findings

**URDF Complexity Spectrum**:

1. **Minimal** (3-5 links): Simple arm or leg chain
   - Pros: Easy to understand, quick to visualize
   - Cons: Not representative of humanoid, oversimplified

2. **Intermediate** (8-12 links): Simplified humanoid (torso, arms, head)
   - Pros: Recognizably humanoid, manageable complexity, teaches key concepts
   - Cons: Still simplified vs production robots

3. **Advanced** (25-40 links): Full humanoid with hands, detailed spine, etc.
   - Pros: Production-realistic, comprehensive
   - Cons: Overwhelming for learners, obscures concepts in details

**Humanoid Robot Reference Models**:
- **PR2**: ~50 links (too complex for education)
- **Rethink Sawyer**: 8 links (single arm, not humanoid)
- **TIAGo**: ~30 links (mobile manipulator, complex)
- **Simple Humanoid Tutorial Models**: 8-12 links (educational sweet spot)

**Educational URDF Requirements**:
- Demonstrate hierarchy (base → torso → arms → end effectors)
- Show multiple joint types (primarily revolute for humanoid)
- Illustrate coordinate frames and transformations
- Include visual, collision, and inertial properties
- Attach sensors (camera, IMU) to show integration
- Load successfully in RViz without overwhelming the view

**Proposed Simplified Humanoid Structure**:
```
base_link (root)
├── torso_link
    ├── head_link
    │   └── camera_link (sensor)
    ├── left_shoulder_link
    │   ├── left_upper_arm_link
    │   │   └── left_lower_arm_link
    │   │       └── left_hand_link
    └── right_shoulder_link
        ├── right_upper_arm_link
        │   └── right_lower_arm_link
                └── right_hand_link

Total: 11 links, 10 joints
```

**Joint Specifications**:
- Neck (head): 1 DOF (yaw or pitch)
- Shoulders: 2 DOF each (pitch, roll) = 2 joints per side
- Elbows: 1 DOF each (pitch)
- Wrists: 1 DOF each (yaw or roll)

### Decision

**Use simplified humanoid URDF with 8-12 links focusing on upper body (torso, head, arms)**

### Rationale

1. **Recognizable**: Clearly humanoid without overwhelming detail
2. **Conceptual Coverage**: Demonstrates all key URDF concepts (links, joints, frames, properties, sensors)
3. **Manageable**: Learners can type, understand, and modify the URDF
4. **Visualizable**: RViz display is clear and uncluttered
5. **Extensible**: Learners can add legs or hands as exercises

### Alternatives Considered

- **Full-body humanoid (30+ links)**: Rejected - too complex, obscures learning points
- **Single arm (5-8 links)**: Rejected - not humanoid, misses bilateral symmetry concepts
- **Stick figure (3-5 links)**: Rejected - too abstract, not representative

### Implementation Notes

- Provide complete URDF file as downloadable artifact
- Use realistic joint limits but simplified inertial properties
- Include both visual (detailed mesh) and collision (simplified geometry) elements
- Add camera on head and IMU on torso to demonstrate sensor integration
- Create launch file to visualize in RViz with joint state publisher GUI

### References

- URDF Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- Example Robot Descriptions: https://github.com/ros/robot_state_publisher

---

## 4. Docusaurus Content Hierarchy

### Research Question
How should multi-chapter content be organized in Docusaurus? What navigation structure best supports progressive learning?

### Findings

**Docusaurus Documentation Patterns** (Source: docusaurus.io/docs):

1. **Flat Structure**: All pages at same level, ordered by metadata
   - Pros: Simple, single sidebar
   - Cons: Poor for hierarchical content, loses context

2. **Hierarchical with Index Pages**: Directories with index.mdx + sub-pages
   - Pros: Clear structure, breadcrumbs, collapsible sections
   - Cons: Requires careful sidebar.js configuration

3. **Single Long Pages**: One page per major topic, anchor links for sections
   - Pros: Single-page reading, good for linear content
   - Cons: Poor discoverability, long load times

**Docusaurus Features Relevant to Educational Content**:
- **Autogenerated Sidebars**: Can auto-generate from directory structure
- **Index Pages**: `index.mdx` serves as category landing page
- **Front Matter**: Control sidebar position, labels, and metadata
- **Admonitions**: `:::note`, `:::warning`, `:::tip` for callouts
- **Code Blocks**: Syntax highlighting with language tags, line numbers, highlighting
- **Tabs**: Multiple code examples or alternatives in single view
- **MDX Components**: Custom interactive elements

**Best Practices for Technical Documentation**:
- **Progressive Disclosure**: Show high-level first, drill down as needed
- **Clear Navigation**: Learners should know where they are and what's next
- **Searchability**: Smaller pages with focused topics improve search results
- **Cross-referencing**: Link related concepts across chapters

**Proposed Structure Analysis**:

```
docs/module-1-ros2/
├── index.mdx                    # Module overview (landing page)
├── 01-architecture/             # Chapter 1 directory
│   ├── index.mdx               # Chapter intro (learning objectives, overview)
│   ├── middleware-role.mdx     # Focused topic pages
│   ├── ros2-overview.mdx
│   └── ...
├── 02-communication/            # Chapter 2 directory
│   ├── index.mdx
│   └── ...
└── ...
```

**Sidebar Configuration** (sidebar.js):
```javascript
module.exports = {
  moduleSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/index',
        {
          type: 'category',
          label: 'Chapter 1: Architecture',
          items: [
            'module-1-ros2/01-architecture/index',
            'module-1-ros2/01-architecture/middleware-role',
            // ... more pages
          ],
        },
        // ... more chapters
      ],
    },
  ],
};
```

### Decision

**Use hierarchical directory structure with index pages and autogenerated sidebars**

### Rationale

1. **Scalability**: Works for single module now, extends to multiple modules later
2. **Navigation**: Clear chapter/section organization with breadcrumbs
3. **Maintenance**: Directory structure reflects learning flow, easy to reorder
4. **Discoverability**: Each focused page is searchable and linkable
5. **Standard Pattern**: Matches Docusaurus best practices and user expectations

### Alternatives Considered

- **Flat structure**: Rejected - loses hierarchical organization, poor for multi-chapter content
- **Single-page chapters**: Rejected - long pages harder to navigate, slower loading
- **Manual sidebar only**: Rejected - error-prone, harder to maintain

### Implementation Details

**Front Matter Standards**:
```yaml
---
sidebar_position: 1
title: Page Title
description: Brief description for SEO
keywords: [ros2, middleware, robotics]
---
```

**Naming Conventions**:
- Directories: `01-architecture`, `02-communication` (numbered for order)
- Files: `middleware-role.mdx`, `ros2-overview.mdx` (kebab-case, descriptive)
- Index files: Always `index.mdx` (Docusaurus convention)

**MDX Component Usage**:
- Code blocks with language tags: ` ```python`, ` ```bash`, ` ```xml`
- Admonitions for important notes: `:::note`, `:::warning`, `:::tip`
- Tabs for alternative approaches: `<Tabs>` component
- Diagrams via Mermaid or static SVG in `/static/img/`

### References

- Docusaurus Docs Structure: https://docusaurus.io/docs/docs-introduction
- Sidebar Configuration: https://docusaurus.io/docs/sidebar
- MDX Features: https://docusaurus.io/docs/markdown-features

---

## 5. ROS 2 Best Practices for Educational Content

### Research Question
What educational approaches and common pitfalls should the module address? How do official tutorials structure learning?

### Findings

**Official ROS 2 Tutorial Structure** (docs.ros.org/en/humble/Tutorials):

1. **Beginner**: CLI tools (ros2 run, ros2 topic, ros2 node)
2. **Beginner Client Libraries**: Writing publishers/subscribers in Python/C++
3. **Intermediate**: Creating packages, custom interfaces, parameters
4. **Advanced**: Launch files, composition, lifecycle

**Common Beginner Pitfalls** (from ROS Answers, community forums):

1. **Domain ID Confusion**: Nodes on different domain IDs can't discover each other
2. **QoS Mismatches**: Publisher and subscriber with incompatible QoS don't connect
3. **Source Setup**: Forgetting to source `/opt/ros/humble/setup.bash`
4. **Workspace Overlay**: Not understanding workspace precedence
5. **Message Type Mismatches**: Trying to subscribe with wrong type
6. **Node Naming**: Duplicate node names cause silent failures
7. **URDF Syntax Errors**: XML validation failures, missing required attributes

**Best Practices to Emphasize**:

1. **Environment Setup**: Always source ROS 2 setup in every terminal
2. **Introspection First**: Use `ros2 node list`, `ros2 topic list` to understand system state
3. **QoS Awareness**: Explain default QoS and when to customize
4. **Error Messages**: Teach how to read and debug common errors
5. **Incremental Testing**: Test each component independently before integration
6. **Workspace Hygiene**: Separate underlay (ROS 2 install) from overlay (workspace)

**Package Structure Convention**:
```
my_robot_package/
├── package.xml          # Package metadata
├── setup.py             # Python package setup
├── setup.cfg            # Python config
├── resource/            # Resource marker
│   └── my_robot_package
├── my_robot_package/    # Python module
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
└── test/                # Tests
    └── test_integration.py
```

**Code Example Pattern**:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Educational Sequence**:
1. Explain concept (what and why)
2. Show minimal example (focus on core pattern)
3. Explain code line-by-line (demystify boilerplate)
4. Run and verify (show expected output)
5. Troubleshoot (address common issues)
6. Extend (suggest modifications for practice)

### Decision

**Follow official ROS 2 tutorial progression with enhanced explanation and troubleshooting guidance**

### Rationale

1. **Alignment**: Consistency with official docs reduces confusion
2. **Proven**: Tutorial structure has been refined through community feedback
3. **Incremental**: Each step builds on previous, no knowledge gaps
4. **Practical**: Combines concepts with hands-on verification
5. **Pitfall-aware**: Proactively addresses common errors

### Implementation Notes

- Include "Common Errors" admonition boxes in relevant sections
- Provide troubleshooting checklist for each major concept
- Show expected output for every command/example
- Explain boilerplate code explicitly (e.g., "Why do we need `rclpy.init()`?")
- Use consistent naming conventions across all examples

### References

- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- ROS Answers (common issues): https://answers.ros.org/

---

## 6. Code Verification Strategy

### Research Question
How should code examples and commands be verified to ensure reproducibility? What tools and processes are needed?

### Findings

**Testing Approaches**:

1. **Manual Execution**: Run each example on target platform, capture output
   - Pros: Validates real user experience
   - Cons: Time-consuming, doesn't scale

2. **Automated CI**: Use GitHub Actions to build and test examples
   - Pros: Catches regressions, scales well
   - Cons: Setup complexity, may differ from user environment

3. **Hybrid**: Manual validation during creation, automated smoke tests for maintenance
   - Pros: Best of both worlds
   - Cons: Requires both manual effort and CI setup

**ROS 2 Validation Tools**:

- **check_urdf**: Validates URDF XML syntax and structure
  ```bash
  check_urdf humanoid.urdf
  ```

- **urdf_to_graphviz**: Visualizes link-joint hierarchy
  ```bash
  urdf_to_graphviz humanoid.urdf
  ```

- **RViz**: Visual validation of URDF and transforms
  ```bash
  ros2 launch my_robot display.launch.py
  ```

- **ros2 run**: Execute nodes to verify functionality
  ```bash
  ros2 run my_package my_node
  ```

- **ros2 topic/service/action**: Introspect running system
  ```bash
  ros2 topic list
  ros2 topic echo /my_topic
  ```

**Verification Checklist Template**:

For each code example:
- [ ] Syntax is valid (linting, no errors)
- [ ] Code runs without errors on Ubuntu 22.04 + ROS 2 Humble
- [ ] Output matches documented expected output
- [ ] Can be stopped cleanly (Ctrl+C, no hanging processes)
- [ ] Appears in ROS 2 graph (`ros2 node list`, `ros2 topic list`)
- [ ] Follows ROS 2 naming conventions

For each URDF:
- [ ] `check_urdf` passes with no errors
- [ ] Loads in RViz without warnings
- [ ] All joints movable with joint_state_publisher_gui
- [ ] TF tree is correct (`ros2 run tf2_tools view_frames`)

For each command:
- [ ] Command exists on Ubuntu 22.04 + ROS 2 Humble
- [ ] Output matches documented output (or explained if variable)
- [ ] Prerequisites clearly stated
- [ ] Error cases documented

**Documentation Requirements**:

For each example, include:
1. **Prerequisites**: What must be installed/sourced/built first
2. **Setup**: Directory structure, file creation
3. **Execution**: Exact command with expected output
4. **Verification**: How to confirm it worked
5. **Cleanup**: How to stop/reset if needed

### Decision

**Use hybrid approach: Manual validation during creation, automated Docusaurus build checks, documented verification steps for readers**

### Rationale

1. **Reproducibility**: Manual testing ensures real user experience
2. **Regression Prevention**: Docusaurus build catches broken links, invalid MDX
3. **Reader Empowerment**: Documented verification steps teach troubleshooting
4. **Pragmatic**: Balances thoroughness with time constraints
5. **Constitution Compliance**: Satisfies "Verification Before Inclusion" principle

### Implementation Process

**Phase 1: Content Creation**
1. Write code example
2. Test on Ubuntu 22.04 + ROS 2 Humble
3. Capture output and screenshots
4. Document in MDX with expected results
5. Add to verification checklist

**Phase 2: Review**
1. Run `npm run build` to validate Docusaurus
2. Check all links (internal and external)
3. Verify all code blocks have language tags
4. Confirm all admonitions render correctly

**Phase 3: Ongoing**
1. Update verification checklist as examples are created
2. Re-test if ROS 2 or Docusaurus versions change
3. Document any platform-specific issues discovered

### Verification Checklist Location

Create `specs/001-ros2-module/verification-checklist.md` with:
- List of all code examples with test status
- List of all commands with verification status
- List of all URDFs with validation status
- Testing environment specification
- Known issues and workarounds

### References

- ROS 2 Testing: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html
- URDF Validation: http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

---

## Research Summary

### Key Decisions Matrix

| Decision Area | Choice | Primary Rationale |
|---------------|--------|-------------------|
| ROS Version | ROS 2 Humble (exclusive) | Future-proof, modern features, LTS support |
| Language | Python (rclpy) | Accessibility, readability, AI/ML alignment |
| URDF Complexity | 8-12 links (simplified humanoid) | Educational clarity without oversimplification |
| Content Structure | Hierarchical Docusaurus with index pages | Scalable, navigable, standard pattern |
| Tutorial Approach | Official ROS 2 progression + enhanced explanation | Alignment with standards, proven pedagogy |
| Verification | Hybrid (manual + build checks + documented steps) | Reproducibility with pragmatic effort |

### Architectural Decisions Requiring ADRs

Per project guidelines, the following decisions meet the three-part significance test (Impact + Alternatives + Scope):

1. **ROS 2 over ROS 1**: Long-term impact, multiple alternatives, affects entire system
2. **Python/rclpy over C++/rclcpp**: Cross-cutting language choice, affects all code examples
3. **URDF Complexity Level**: Influences content depth and reader experience
4. **Docusaurus Hierarchical Structure**: Affects navigation, maintenance, and extensibility

**Recommendation**: After plan approval, run `/sp.adr` for each decision to document rationale, alternatives, and consequences formally.

### Open Questions

1. **Diagram Creation**: How will architectural diagrams be created? (Mermaid, draw.io, code-generated?)
   - Recommendation: Research Mermaid for simple diagrams, SVG export for complex ones

2. **Interactive Elements**: Should examples include interactive code playgrounds?
   - Recommendation: Out of scope for v1, evaluate for future enhancement

3. **Video Content**: Should any sections include video demonstrations?
   - Recommendation: Out of scope, focus on text + static diagrams for reproducibility

### Next Steps

1. Proceed to Phase 1: Design & Contracts
2. Create `data-model.md` defining content structure entities
3. Create `contracts/` with detailed chapter specifications
4. Create `quickstart.md` for rapid ROS 2 reference
5. Prepare for task generation (`/sp.tasks`)

---

**Document Status**: Complete ✅
**All Research Objectives Resolved**: Yes
**Ready for Phase 1**: Yes

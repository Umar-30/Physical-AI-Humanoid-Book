// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to building intelligent humanoid robots',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config
  organizationName: 'your-org',
  projectName: 'humanoid-robotics-with-ai',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
  },

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Remove this to remove the "edit this page" links.
          editUrl: undefined,
        },
        blog: false, // Disable blog for this educational content
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/modules',
            position: 'left',
            label: '📚 Modules',
          },
          {
            type: 'doc',
            docId: 'module-1-ros2/index',
            position: 'left',
            label: 'Module 1',
          },
          {
            type: 'doc',
            docId: 'module-2-digital-twin/module-2-overview',
            position: 'left',
            label: 'Module 2',
          },
          {
            href: 'https://github.com/your-org/humanoid-robotics-with-ai',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System (ROS 2)',
                to: '/docs/module-1-ros2',
              },
              {
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
                to: '/docs/module-2',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Gazebo Harmonic Docs',
                href: 'https://gazebosim.org/docs/harmonic',
              },
              {
                label: 'Unity Robotics Hub',
                href: 'https://github.com/Unity-Technologies/Unity-Robotics-Hub',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/your-org/humanoid-robotics-with-ai',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Humanoid Robotics with AI. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['bash', 'python'],
      },
    }),
};

module.exports = config;

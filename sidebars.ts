import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics book
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'preface',
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      link: {
        type: 'doc',
        id: 'chapter1/chapter1',
      },
      items: [
        'chapter1/chapter1-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter1/sections/chapter1-sections-1-1-defining-physical-ai',
            'chapter1/sections/chapter1-sections-1-2-historical-evolution',
            'chapter1/sections/chapter1-sections-1-3-humanoid-imperative',
            'chapter1/sections/chapter1-sections-1-4-core-components',
            'chapter1/sections/chapter1-sections-1-5-ethical-implications',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: Essential Mathematics',
      link: {
        type: 'doc',
        id: 'chapter2/chapter2-index',
      },
      items: [
        'chapter2/chapter2-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter2/sections/chapter2-sections-2-1-linear-algebra',
            'chapter2/sections/chapter2-sections-2-2-calculus',
            'chapter2/sections/chapter2-sections-2-3-quaternions',
            'chapter2/sections/chapter2-sections-2-4-probability-statistics',
            'chapter2/sections/chapter2-sections-2-5-control-theory-math',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Kinematics and Dynamics',
      link: {
        type: 'doc',
        id: 'chapter3/chapter3',
      },
      items: [
        'chapter3/chapter3-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter3/sections/chapter3-sections-3-1-rigid-body-transformations',
            'chapter3/sections/chapter3-sections-3-2-forward-kinematics',
            'chapter3/sections/chapter3-sections-3-3-inverse-kinematics',
            'chapter3/sections/chapter3-sections-3-4-differential-kinematics',
            'chapter3/sections/chapter3-sections-3-5-robot-dynamics',
            'chapter3/sections/chapter3-sections-3-6-balance-stability',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: Sensing and Perception',
      link: {
        type: 'doc',
        id: 'chapter4/chapter4-index',
      },
      items: [
        'chapter4/chapter4-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter4/sections/4.1-proprioception',
            'chapter4/sections/chapter4-sections-4-2-exteroception',
            'chapter4/sections/chapter4-sections-4-3-tactile-sensing',
            'chapter4/sections/chapter4-sections-4-4-auditory-perception',
            'chapter4/sections/chapter4-sections-4-5-sensor-fusion',
            'chapter4/sections/chapter4-sections-4-6-computer-vision-fundamentals',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Actuation and Mechanical Design',
      link: {
        type: 'doc',
        id: 'chapter5/chapter5',
      },
      items: [
        'chapter5/chapter5-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter5/sections/chapter5-sections-5-1-electric-motors',
            'chapter5/sections/chapter5-sections-5-2-hydraulic-pneumatic',
            'chapter5/sections/chapter5-sections-5-3-series-elastic-actuators',
            'chapter5/sections/chapter5-sections-5-4-gearboxes-transmission',
            'chapter5/sections/chapter5-sections-5-5-mechanical-structures',
            'chapter5/sections/chapter5-sections-5-6-thermal-power',
          ],
        },
      ],
    },
  ],
};

export default sidebars;

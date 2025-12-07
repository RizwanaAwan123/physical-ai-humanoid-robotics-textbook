import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics book
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    {
      type: 'doc',
      id: 'title-page',
      label: 'Title Page',
    },
    {
      type: 'doc',
      id: 'preface',
      label: 'Preface',
    },
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      link: {
        type: 'doc',
        id: 'chapter1/chapter1-index',
      },
      items: [
        'chapter1/chapter1-learning-objectives',
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
            'chapter2/sections/2.1-linear-algebra',
            'chapter2/sections/2.2-calculus',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Kinematics and Dynamics',
      link: {
        type: 'doc',
        id: 'chapter3/chapter3-index',
      },
      items: [
        'chapter3/chapter3-learning-objectives',
        {
          type: 'category',
          label: 'Sections',
          items: [
            'chapter3/sections/3.1-rigid-body-transformations',
            'chapter3/sections/3.2-forward-kinematics',
            'chapter3/sections/3.3-inverse-kinematics',
            'chapter3/sections/3.4-differential-kinematics',
            'chapter3/sections/3.5-robot-dynamics',
            'chapter3/sections/3.6-balance-stability',
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
            'chapter4/sections/4.2-exteroception',
            'chapter4/sections/4.3-tactile-sensing',
            'chapter4/sections/4.4-auditory-perception',
            'chapter4/sections/4.5-sensor-fusion',
            'chapter4/sections/4.6-computer-vision-fundamentals',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Actuation and Mechanical Design',
      link: {
        type: 'doc',
        id: 'chapter5/chapter5-index',
      },
      items: [
        'chapter5/chapter5-learning-objectives',
      ],
    },
  ],
};

export default sidebars;

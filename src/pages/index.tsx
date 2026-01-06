import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--secondary button--lg', styles.primaryButton)}
            to="docs/">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--secondary button--lg', styles.secondaryButton)}
            to="docs/quarter-overview">
            View Overview
          </Link>
        </div>
      </div>
    </header>
  );
}

function BookOverview() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className={styles.textCenter}>
              <Heading as="h2" className={styles.sectionTitle}>About This Book</Heading>
              <div className={styles.bookDescription}>
                <p className="text--left">
                  This comprehensive curriculum explores Physical AI - AI that lives in the real world and understands physical laws.
                  Students will design, simulate, and deploy humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac.
                  This module-based approach provides hands-on experience with the complete Physical AI ecosystem.
                </p>
              </div>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '3rem', gap: '2rem'}}>
          <div className="col col--4">
            <div className={styles.metricCard}>
              <div className={styles.metricIcon}>
                <svg viewBox="0 0 24 24" width="36" height="36">
                  <path fill="white" d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z"/>
                </svg>
              </div>
              <Heading as="h3" className={styles.metricTitle}>4 Core Modules</Heading>
              <p className={styles.metricDescription}>Complete curriculum from ROS 2 fundamentals to Vision-Language-Action systems</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.metricCard}>
              <div className={styles.metricIcon}>
                <svg viewBox="0 0 24 24" width="36" height="36">
                  <path fill="white" d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                </svg>
              </div>
              <Heading as="h3" className={styles.metricTitle}>Capstone Project</Heading>
              <p className={styles.metricDescription}>Autonomous humanoid robot integrating all modules and technologies</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.metricCard}>
              <div className={styles.metricIcon}>
                <svg viewBox="0 0 24 24" width="36" height="36">
                  <path fill="white" d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm3.5 6L12 10.5 8.5 8 12 5.5 15.5 8zM8.5 16L12 13.5 15.5 16 12 18.5 8.5 16z"/>
                </svg>
              </div>
              <Heading as="h3" className={styles.metricTitle}>Industry-Focused</Heading>
              <p className={styles.metricDescription}>Real-world applications using NVIDIA Isaac, ROS 2, and modern robotics technologies</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesList() {
  const modules = [
    {
      title: 'Module 1: The Robotic Nervous System (ROS 2)',
      description: 'Why ROS 2 is the nervous system of a robot - Nodes, Topics, Services, Actions, Python control using rclpy, connecting AI agents to ROS controllers, URDF for humanoid robots',
      link: 'docs/module1/introduction',
    },
    {
      title: 'Module 2: The Digital Twin (Gazebo & Unity)',
      description: 'Physics-based simulation, Gazebo environments, gravity, collisions, joints, sensor simulation, Unity for visualization and human-robot interaction',
      link: 'docs/module2/introduction',
    },
    {
      title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      description: 'Isaac Sim and synthetic data, Isaac ROS pipelines, Visual SLAM (VSLAM), Nav2 for humanoid navigation, training and sim-to-real concepts',
      link: 'docs/module3/introduction',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Voice-to-action robotics, Whisper for speech recognition, LLM-based planning (natural language → ROS actions), vision + manipulation loop, cognitive robotics concepts',
      link: 'docs/module4/introduction',
    },
    {
      title: 'Capstone Project: The Autonomous Humanoid',
      description: 'Integrating all modules: voice commands, task planning using LLMs, navigation with ROS 2 + Nav2, object detection with vision, manipulation in simulation',
      link: 'docs/capstone-project',
    },
  ];

  return (
    <section className={styles.chaptersSection}>
      <div className="container">
        <div className="text--center">
          <Heading as="h2" className={styles.sectionTitle}>Module Curriculum</Heading>
        </div>
        <div className={styles.chaptersGrid}>
          {modules.map((module, idx) => (
            <Link
              key={idx}
              to={module.link}
              className={styles.chapterCard}
              style={{
                animationDelay: `${idx * 0.15}s`
              }}
            >
              <div className={styles.chapterContent}>
                <div className={styles.chapterHeader}>
                  <Heading as="h3" className={styles.chapterTitle}>{module.title}</Heading>
                </div>
                <p className={styles.chapterDescription}>{module.description}</p>
                <div className={styles.chapterFooter}>
                  <span className={styles.readLink}>Explore Module →</span>
                </div>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A Comprehensive Guide to Physical AI and Humanoid Robotics - covering ROS 2, NVIDIA Isaac, Gazebo simulation, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <BookOverview />
        <ModulesList />
      </main>
    </Layout>
  );
}

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
            to="docs/preface">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--secondary button--lg', styles.secondaryButton)}
            to="docs/preface">
            View Preface
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
            <div className="text--center padding-horiz--md">
              <Heading as="h2">About This Book</Heading>
              <p className="text--left" style={{maxWidth: '800px', margin: '0 auto', fontSize: '1.1rem'}}>
                This comprehensive guide explores the fascinating intersection of artificial intelligence
                and physical robotics, focusing on the design, development, and deployment of humanoid robots.
                From fundamental mathematics to advanced control systems, this book provides a complete
                foundation for understanding Physical AI and humanoid robotics.
              </p>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '3rem'}}>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">5 Chapters</Heading>
              <p>Comprehensive coverage from introduction to advanced topics in humanoid robotics</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">Multiple Sections</Heading>
              <p>Detailed sections covering mathematics, kinematics, perception, and mechanical design</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <Heading as="h3">Practical Focus</Heading>
              <p>Real-world applications and implementations of Physical AI concepts</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ChaptersList() {
  const chapters = [
    {
      title: 'Chapter 1: Introduction to Physical AI',
      description: 'Defining Physical AI, historical evolution, and the humanoid imperative',
      link: 'docs/chapter1',
    },
    {
      title: 'Chapter 2: Essential Mathematics',
      description: 'Linear algebra, calculus, quaternions, and control theory fundamentals',
      link: 'docs/chapter2-index',
    },
    {
      title: 'Chapter 3: Kinematics and Dynamics',
      description: 'Forward and inverse kinematics, robot dynamics, balance, and stability',
      link: 'docs/chapter3',
    },
    {
      title: 'Chapter 4: Sensing and Perception',
      description: 'Proprioception, exteroception, sensor fusion, and computer vision',
      link: 'docs/chapter4-index',
    },
    {
      title: 'Chapter 5: Actuation and Mechanical Design',
      description: 'Electric motors, actuators, mechanical structures, and power systems',
      link: 'docs/chapter5',
    },
  ];

  return (
    <section className={styles.chaptersSection}>
      <div className="container">
        <div className="text--center">
          <Heading as="h2" className={styles.sectionTitle}>Table of Contents</Heading>
        </div>
        <div className={styles.chaptersGrid}>
          {chapters.map((chapter, idx) => (
            <Link
              key={idx}
              to={chapter.link}
              className={styles.chapterCard}
              style={{
                animationDelay: `${idx * 0.15}s`
              }}
            >
              <div className={styles.chapterContent}>
                <div className={styles.chapterHeader}>
                  <Heading as="h3" className={styles.chapterTitle}>{chapter.title}</Heading>
                </div>
                <p className={styles.chapterDescription}>{chapter.description}</p>
                <div className={styles.chapterFooter}>
                  <span className={styles.readLink}>Read Chapter →</span>
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
      description="A Comprehensive Guide to Physical AI and Humanoid Robotics - covering mathematics, kinematics, dynamics, perception, and mechanical design">
      <HomepageHeader />
      <main>
        <BookOverview />
        <ChaptersList />
      </main>
    </Layout>
  );
}

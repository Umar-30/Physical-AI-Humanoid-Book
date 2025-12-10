import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

const HomepageHero = () => {
  const { siteConfig } = useDocusaurusContext();

  const bookTitle = "Physical AI & Humanoid Robotics";
  const welcomeQuote = "“Welcome to the frontier of intelligent machines — where physical AI meets human-like robotics.”";
  const subtitle = "A Hands-On Guide to Building AI-Powered Physical Systems and Humanoid Robotics — The Spec & Co-Learning Way";

  const bookCoverPath = "https://media.licdn.com/dms/image/v2/D4E12AQEpoNugVTA3Dg/article-cover_image-shrink_720_1280/article-cover_image-shrink_720_1280/0/1737345110577?e=2147483647&v=beta&t=a8FgoA4EnzzXRps-byiIXvOwa72TAqOypsLQVZyX3eo";

  const badges = ["Open Source", "Co-Learning with AI", "Spec-Driven Development"];

  const ctas = [
    { label: "Start Learning →", to: "/learning", primary: true },
  ];

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={clsx("container", styles.heroContainer)}>
        <div className={styles.twoColumnLayout}>
          
          {/* Left Column — Book Image */}
          <div className={styles.bookCoverCard}>
            <img
              src={bookCoverPath}
              alt={bookTitle}
              className={styles.bookCover}
            />
          </div>

          {/* Right Column — Text */}
          <div className={styles.rightColumnContent}>
            <h1 className={styles.bookTitle}>{bookTitle}</h1>
            <p className={styles.welcomeQuoteText}>{welcomeQuote}</p>
            <p className={styles.bookSubtitle}>{subtitle}</p>

            <div className={styles.separator} />

            <div className={styles.badgesContainer}>
              {badges.map((badge, index) => (
                <span key={index} className={styles.badgePill}>
                  {badge}
                </span>
              ))}
            </div>

            <div className={styles.ctaButtonsContainer}>
              {ctas.map((cta, index) => (
                <Link
                  key={index}
                  className={clsx(
                    'button',
                    cta.primary ? styles.ctaPrimary : styles.ctaSecondary,
                    styles.ctaButtonCommon
                  )}
                  to={cta.to}
                >
                  {cta.label}
                </Link>
              ))}
            </div>

          </div>
        </div>
      </div>
    </header>
  );
};

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A Hands-On Guide to Building AI-Powered Physical Systems and Humanoid Robotics — The Spec & Co-Learning Way"
      noFooter={true}   // FOOTER REMOVED
    >
      <HomepageHero />
      <main />
    </Layout>
  );
}

import React from 'react';
import Layout from '@theme/Layout';
import CourseSection from '../components/CourseSection'; // Path relative to learning.tsx

export default function LearningPage(): JSX.Element {
  return (
    <Layout
      title="Start Learning"
      description="Begin your journey into Physical AI & Humanoid Robotics">
      <main>
        <CourseSection />
      </main>
    </Layout>
  );
}
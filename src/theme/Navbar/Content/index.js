/**
 * Navbar Content Component (Swizzled)
 *
 * Wraps the default navbar content to add the LanguageToggle button
 * as a right-side navbar item.
 */

import React from 'react';
import Content from '@theme-original/Navbar/Content';
import LanguageToggle from '../../../components/LanguageToggle';
import styles from './styles.module.css';

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <div className={styles.languageToggleWrapper}>
        <LanguageToggle />
      </div>
    </>
  );
}

# Research Findings: Docusaurus Build Performance

## 1. Docusaurus Build Performance & Optimization Strategies

### Decision: Prioritize Docusaurus Native Optimizations and Web Best Practices

### Rationale:
To ensure efficient development and deployment of the book, optimizing Docusaurus build times is crucial. Leveraging Docusaurus's built-in experimental features, alongside general web performance best practices, will provide the best balance of speed and maintainability. Given the context of a book chapter, excessive focus on extreme performance tuning is not required, but a smooth build process is desirable.

### Alternatives Considered:
- **No specific optimization**: Rejected due to potential for slow build times, especially as the book grows, impacting developer experience and CI/CD.
- **Deep dive into Webpack/Rspack configurations**: Rejected for this phase as Docusaurus abstracts much of this, and the experimental features (like `experimental_faster`) offer high-level control that is sufficient for initial planning. More granular tuning can be considered if build times become a significant bottleneck later.

### Key Optimization Strategies Identified:

#### a. Docusaurus Faster Project (Experimental)
- **`experimental_faster`**: Enable in `docusaurus.config.js` for Docusaurus v3.6+. Utilizes Rspack, SWC, and Lightning CSS. Expected to provide 2-4x faster production builds. This will be the primary lever for build speed.
- **Individual Faster Options**: Specific components like `swcJsLoader`, `swcJsMinimizer`, `swcHtmlMinimizer`, `lightningCssMinimizer`, and `rspackBundler` can be enabled individually if fine-grained control is needed.

#### b. Docusaurus Version Updates
- **`ssgWorkerThreads` (v3.8+)**: Uses Node.js Worker threads for Static Site Generation (SSG), potentially reducing SSG time by ~2x. This is relevant for CPU utilization during content generation.
- **`rspackPersistentCache` (v3.8+)**: When using Rspack, this feature can make rebuilds 2-5x faster by reusing computations. Important for incremental builds and CI/CD caching.
- **`mdxCrossCompilerCache`**: Compiles MDX files once for both browser and Node.js environments, reducing redundant compilation.

#### c. General Web Performance Techniques (Asset Optimization)
- **Image Compression & Lazy Loading**: Convert images to modern formats (WebP/AVIF) and implement `loading="lazy"` for markdown images. This is directly applicable to diagram placement strategy.
- **Bundle Optimization**: While Docusaurus manages much of this, being mindful of large dependencies and using dynamic imports for non-critical components is a general best practice.
- **Font Subsetting & Preloading**: Relevant if custom fonts are introduced.

#### d. Caching for Rebuilds
- **Persistent Caching**: Ensure `node_modules/.cache` is persisted across builds, especially in CI/CD, to leverage Webpack's persistent caching.

#### e. Monitoring and Debugging
- **`DOCUSAURUS_PERF_LOGGER=true`**: Environment variable to get verbose build step timings, useful for identifying bottlenecks if they arise.
- **Rsdoctor Plugin**: Can analyze bundling for slow loaders/plugins.

## Conclusion for "Performance Goals" in Technical Context:
While no strict numerical performance goals are set for the build, the objective is to maintain a "fast and efficient" build process. This will be achieved by enabling Docusaurus's `experimental_faster` feature and ensuring proper caching. Monitoring with `DOCUSAURUS_PERF_LOGGER` will be used if performance degrades.

---
**Status**: Resolved
**Decision**: Implement Docusaurus's `experimental_faster` feature (v3.6+) and potentially `ssgWorkerThreads` (v3.8+) and `rspackPersistentCache` (v3.8+). Apply general web performance best practices for images and assets.

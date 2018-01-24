/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */


const siteConfig = {
  title: 'Udacidrone' /* title for your website */,
  tagline: 'An API for working with flying objects, simulated, unidentified and otherwise.',
  url: 'https://udacity.github.io/udacidrone' /* your website url */,
  sourceCodeButton: null,
  baseUrl: '/udacidrone/',
  headerLinks: [
    {doc: 'getting-started', label: 'Docs'},
    {languages: true},
    {search: true},
    { href: "https://github.com/udacity/udacidrone", label: "GitHub" }
    // {blog: true, label: 'Blog'},
  ],
  /* path to images for header/footer */
  // headerIcon: 'img/docusaurus.svg',
  // footerIcon: 'img/docusaurus.svg',
  // favicon: 'img/favicon.png',
  /* colors for website */
  colors: {
    primaryColor: '#02B3E4',
    secondaryColor: '#02CCBA',
    codeColor: "rgba(243, 136, 136, 0.03)"
  },
  users: [],
  // This copyright info is used in /core/Footer.js and blog rss/atom feeds.
  copyright:
    'Copyright © ' +
    new Date().getFullYear() +
    'Udacity',
  // These have to match the github organization and repo names
  organizationName: 'udacity', // or set an env variable ORGANIZATION_NAME
  projectName: 'udacidrone', // or set an env variable PROJECT_NAME
  highlight: {
    // Highlight.js theme to use for syntax highlighting in code blocks
    theme: 'default',
  },
  scripts: ['https://buttons.github.io/buttons.js'],
};

module.exports = siteConfig;

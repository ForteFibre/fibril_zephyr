# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------

project = 'fibril_zephyr'
copyright = '2026, ForteFibre'
author = 'ForteFibre'
release = '1.0.0'

# -- General configuration ---------------------------------------------------

extensions = [
    'myst_parser',
    'sphinx.ext.intersphinx',
]

templates_path = ['_templates']

# _doxygen holds the Doxygen entry pages, which Doxygen renders on its own.
# Without excluding it, myst_parser picks main.md up as a Sphinx document that
# no toctree references.
exclude_patterns = [
    '_build*',
    '_doxygen',
    'Thumbs.db',
    '.DS_Store',
]

# -- Options for MyST --------------------------------------------------------

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]

# Guide pages are written flat, without a document title separate from the
# first heading, so only the top level becomes an anchor target.
myst_heading_anchors = 3

# -- Options for HTML output -------------------------------------------------

html_theme = 'alabaster'

# -- Options for Intersphinx -------------------------------------------------

intersphinx_mapping = {'zephyr': ('https://docs.zephyrproject.org/latest/', None)}

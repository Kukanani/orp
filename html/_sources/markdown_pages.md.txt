# Markdown Formatting
This page is meant for developers, documenters, and maintainers of ORP.
This page explains the syntax recognized by Sphinx in order to properly display information.

As a side note, both markdown and rst are enabled in the conf.py, allowing some flexibility.


## Recommonmark
Recommonmark is a bridge to displaying CommonMark, a syntax specification for Markdown. The full specifications of CommonMark are outlined
[here](https://spec.commonmark.org/0.28/).
This [link](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet) refers an easy markdown cheatsheet. There are no guarantees that Sphinx CommonMark has the same specifications.

## Setup
After creating your new markdown/rst files with your information or documentation, make sure to add it to the index.rst toctree.
In addition, to build this project, go to the parent directory orp/sphinx-docs and call the following command: ``sphinx-build -b html source build.``

Build can be replaced by another directory (i.e. for testing purposes).
***
The following are basic structural syntax for formatting orp guides or documentation.
## Formatting
### Headings
\#

\#\#

\#\#\#

\#\#\#\#

\#\#\#\#\#

Followed by a space and text act as headings, from largest to smallest, respectively.

**IMPORTANT**: *These headings (\# and \#\# are recognized by Sphinx and are inserted into the table of contents. By default, the maxdepth for the index.rst is limited to 2, hiding all subheadings that are \#\#\#, \#\#\#\#, and \#\#\#\#\#.*

##### This is a \#\#\#\#\# heading!

### Lists
- '-' followed by a space and then text is treated as an element of a bulleted list.
    - el
    - el
    - el
- similarly, a number with a period and a space then text is treated an element of a numbered list.
    1. el1
    2. el2
    3. el3

'\*\*\*' acts a thematic break.
***

Author: Matthew Yu
Last Modified: 03/15/19

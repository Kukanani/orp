# ORP Sphinx Documentation README
##### Author: Matthew Yu
##### Last Modified: March 8th, 2019

----------------------------------

Sphinx version 1.8.4
Markdown support:
https://www.sphinx-doc.org/en/stable/usage/markdown.html


Publishing Directions (For Adam Allevato and/or collaborators of ORP):
##### First time setup
    1. On the orp github code page, go to Settings -> GitHub Pages.
    2. Select Source -> master branch /docs folder.

##### Publishing updates or documentation changes
To build the most recent version of the docs, use the following command in the root directory (orp/sphinx-docs):
**$ sphinx-build -b html source build**
Build is the directory that you want the resulting html files to be placed.

_This project is structured such that multiple builds and sources can be configured (i.e. for a testing and stable version, etc.)_

Alternatively, you can use ``make html``, which is preconfigured to update the docs subdirectory in the parent directory. This subdirectory is directly linked to the gh-pages, which updates when pushed.

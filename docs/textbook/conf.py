from datetime import date


project = "SummerSLAM"
author = "Thomas Pan"
copyright = f"{date.today().year}, Thomas Pan"

extensions = [
    "myst_parser",
    "sphinx.ext.mathjax",
]

source_suffix = {".md": "markdown"}
master_doc = "index"
language = "en"
exclude_patterns = ["_build", ".doctrees", "Thumbs.db", ".DS_Store"]
templates_path = ["_templates"]

myst_enable_extensions = [
    "amsmath",
    "colon_fence",
    "deflist",
    "dollarmath",
    "tasklist",
]
myst_heading_anchors = 4

html_theme = "sphinx_rtd_theme"
html_title = "SummerSLAM Engineering Notes"
html_baseurl = "https://tuomaaa.github.io/XDriveSLAMRover/"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_js_files = ["footer-license.js"]
html_show_sourcelink = True
html_show_sphinx = True

html_theme_options = {
    "collapse_navigation": False,
    "navigation_depth": 3,
    "prev_next_buttons_location": "bottom",
    "sticky_navigation": True,
    "style_external_links": True,
    "titles_only": False,
}

html_context = {
    "display_github": True,
    "github_user": "Tuomaaa",
    "github_repo": "XDriveSLAMRover",
    "github_version": "main",
    "conf_py_path": "/docs/textbook/",
}

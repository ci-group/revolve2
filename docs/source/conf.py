# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Revolve2"
copyright = (
    "Computational Intelligence Group, Vrije Universiteit Amsterdam & Contributors"
)
author = "Computational Intelligence Group, Vrije Universiteit Amsterdam & Contributors"
release = "1.0.0rc1-test"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "autoapi.extension",
    "sphinx.ext.autosectionlabel",
]

templates_path = ["_templates"]
exclude_patterns = []
add_module_names = False

# -- Autoapi extension -------------------------------------------------------

with open("../../packages.txt", "r") as f:
    autoapi_dirs = [line.strip() for line in f.readlines()]
    autoapi_dirs = [
        f"../../{package}"
        for package in autoapi_dirs
        if not package.startswith("examples/")
    ]

autoapi_options = [
    "members",
    "undoc-members",
    "special-members",
    "show-inheritance",
    "show-inheritance-diagram",
    "imported-members",
    "show-module-summary",
    "titles_only=True",
]
autoapi_add_toctree_entry = True
autoapi_template_dir = "_templates/autoapi"

# -- Autosectionlabel extensio -----------------------------------------------

autosectionlabel_prefix_document = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_logo = "logo.png"
html_favicon = "favicon.png"

html_show_sourcelink = False
html_theme_options = {
    "prev_next_buttons_location": None,
    "collapse_navigation": False,
}

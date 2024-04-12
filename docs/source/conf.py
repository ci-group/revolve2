# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import yaml

project = "Revolve2"
copyright = (
    "Computational Intelligence Group, Vrije Universiteit Amsterdam & Contributors"
)
author = "Computational Intelligence Group, Vrije Universiteit Amsterdam & Contributors"
release = "1.0.1"

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

with open("../../project.yml") as file:
    data = yaml.safe_load(file)
    namespace = data["revolve2-namespace"]
    platform_dependent = [
        f"../../{pkg}/{namespace}" for pkg in data["platform_dependent_packages"]
    ]
    platform_independent = [
        f"../../{pkg}/{namespace}" for pkg in data["platform_independent_packages"]
    ]
    autoapi_dirs = platform_dependent + platform_independent

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

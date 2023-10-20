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
release = "v0.4.2-beta2"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["autoapi.extension", "sphinx.ext.autosectionlabel"]

templates_path = ["_templates"]
exclude_patterns = []

# -- Autoapi extension -------------------------------------------------------

autoapi_dirs = [
    "../../simulation/revolve2",
    "../../modular_robot/revolve2",
    "../../modular_robot_simulation/revolve2",
]
autoapi_options = [
    "members",
    "undoc-members",
    "special-members",
    "show-inheritance",
    "show-inheritance-diagram",
    "imported-members",
    "show-module-summary",
]
autoapi_add_toctree_entry = True

# -- autosectionlabel extensio -----------------------------------------------

autosectionlabel_prefix_document = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_logo = "logo.png"
html_favicon = "favicon.png"

html_static_path = ["_static"]

html_show_sourcelink = False
html_theme_options = {
    "prev_next_buttons_location": None,
    # "titles_only": True,
    # "collapse_navigation": False,
}

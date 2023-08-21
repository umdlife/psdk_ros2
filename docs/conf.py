from exhale import utils

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "psdk_ros2 wrapper"
copyright = "2023, Unmanned Life"
author = "Bianca Bendris"
master_doc = "index"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["breathe", "exhale", "myst_parser"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"

# -- Breathe configuration -------------------------------------------------

breathe_projects = {"C++ API Documentation": "doxyoutput/xml"}
breathe_default_project = "C++ API Documentation"


# somewhere in `conf.py`, *BERORE* declaring `exhale_args`
def specificationsForKind(kind):
    """
    For a given input ``kind``, return the list of reStructuredText specifications
    for the associated Breathe directive.
    """
    # Change the defaults for .. doxygenclass:: and .. doxygenstruct::
    if kind == "class" or kind == "struct":
        return [":members:", ":protected-members:", ":private-members:"]
    # An empty list signals to Exhale to use the defaults
    else:
        return []


# Setup the exhale extension
exhale_args = {
    "verboseBuild": True,
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "Library API",
    "doxygenStripFromPath": "..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleUseDoxyfile": True,
    "customSpecificationsMapping": utils.makeCustomSpecificationsMapping(
        specificationsForKind
    ),
}


# Tell sphinx what the primary language being documented is.
primary_domain = "cpp"

# Tell sphinx what the pygments highlight language should be.
highlight_language = "cpp"

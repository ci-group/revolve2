# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SPHINXAPIDOC  ?= sphinx-apidoc
APIDOCARGS    ?= -o source/api_reference --implicit-namespaces --module-first --separate --tocfile index --doc-project 'API Reference' --maxdepth 1 --templatedir $(THIS_DIR)/source/apidoc_template
SOURCEDIR     = source
BUILDDIR      = build

THIS_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

apidoc:
	@$(SPHINXAPIDOC) ../ci_group/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../experimentation/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../modular_robot/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../modular_robot_simulation/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../simulation/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../simulators/mujoco/revolve2 $(APIDOCARGS)

apidoc_overwrite:
	APIDOCARGS="$(APIDOCARGS) -f" $(MAKE) apidoc

.PHONY: help Makefile apidoc apidoc_overwrite

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

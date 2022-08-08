# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SPHINXAPIDOC  ?= sphinx-apidoc
APIDOCARGS    ?= -o source/api_reference --implicit-namespaces --tocfile index --doc-project 'API Reference' --maxdepth 1 --templatedir $(THIS_DIR)/source/apidoc_template
SOURCEDIR     = source
BUILDDIR      = build

THIS_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

apidoc:
	@$(SPHINXAPIDOC) ../core/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../genotypes/cppnwin/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../runners/isaacgym/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../serialization/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../actor_controller/revolve2 $(APIDOCARGS)
	@$(SPHINXAPIDOC) ../rpi_controller/revolve2 $(APIDOCARGS)

apidoc_overwrite:
	APIDOCARGS="$(APIDOCARGS) -f" $(MAKE) apidoc

.PHONY: help Makefile apidoc apidoc_overwrite

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

web-and-serve: html
	cd _build/html && python3 -m http.server 8080

web-and-publish: html
	rm -fr /tmp/ob-web/ && mkdir /tmp/ob-web/ && cd /tmp/ob-web/ && \
	git clone -b html https://github.com/open-beam/openbeam.git openbeam-web && \
	rm -fr openbeam-web/* ; rm -fr openbeam-web/.gitignore
	cp -vr _build/html/* /tmp/ob-web/openbeam-web/
	cd /tmp/ob-web/openbeam-web/ && git add *
	cd /tmp/ob-web/openbeam-web/ && git commit -am "html" --amend
	cd /tmp/ob-web/openbeam-web/ && git push origin html --force


# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

PDF_FILE=calibration.pdf
TEX_FILE=calibration.tex
FLAGS=--bibtex-cmd=biber --latex-cmd pdflatex
LATEX_OUT_DIR=.latex.out

.PHONY: FORCE
$(PDF_FILE): FORCE
	@latexrun $(FLAGS) $(TEX_FILE) -O $(LATEX_OUT_DIR) -o $(PDF_FILE)

.PHONY: clean
clean:
# Remove the old PDF.
# Leading dash causes processing to proceed even if errors occur.
	@-rm -f $(PDF_FILE)
	@-rm -rf $(LATEX_OUT_DIR)

#-----------------------------------------------------------------------
# cmakefile

PROJECT_PATH = $(realpath .)/
OUTDATA_PATH = $(PROJECT_PATH)/outdata

TARGET 		= documentation_ws24.pdf
TARGET_PATH	= $(OUTDATA_PATH)/$(TARGET)


PDF_FLAG			= -shell-escape -output-directory=$(OUTDATA_PATH)


all: $(TARGET)

$(TARGET):
	mkdir $(OUTDATA_PATH) 
	pdflatex $(PDF_FLAG) $(TARGET:.pdf=.tex)
	cd $(OUTDATA_PATH)/ && set BIBINPUTS="$(PROJECT_PATH);%BIBINPUTS%" && set BSTINPUTS="$(PROJECT_PATH);%BSTINPUTS%" && bibtex $(TARGET:.pdf=)
	pdflatex $(PDF_FLAG) $(PDF_FLAG) $(TARGET:.pdf=.tex)
	pdflatex $(PDF_FLAG) $(PDF_FLAG) $(TARGET:.pdf=.tex)
	cp $(TARGET_PATH) $(PROJECT_PATH)


#-----------------------------------------------------------------------
# clean
clean:
	rm -f $(addprefix $(TARGET), *.pdf *.toc *.pyg *.out *.lot *.pyg *.out *.lof *.aux *.acr *.bib *.log)
	rm -f $(TARGET)
	rm -rf $(OUTDATA_PATH)
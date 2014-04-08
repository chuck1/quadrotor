
all:
	latex files/main.tex
	makeindex main.nlo -s nomencl.ist -o main.nls
	pdflatex files/main.tex
	#dvipdfm main.dvi


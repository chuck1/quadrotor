
all:
	latex -output-directory=latex files/main.tex
	makeindex latex/main.nlo -s nomencl.ist -o latex/main.nls
	pdflatex -output-directory=latex files/main.tex
	#dvipdfm main.dvi


For this project there is only one set of input files (regardless
of platform) for two reasons:
1) It appears that a patch at some point made Visual Studio able
   to read Unix-format text files correctly, using getline(),
   cin.get(), etc.  If you're using Visual Studio, make sure that
   you're using 2017 or 2019, and that Windows has no patches to do.
2) This project shouldn't need getline() at all; you should be
   reading all of your input with >>.

There are five input files: spec-test.txt, sample-ab.txt,
sample-c.txt, sample-d.txt, and sample-e.txt.
  spec-test.txt has 5 vertices (from the project spec)
  sample-ab.txt has 10000 vertices, sample-c.txt and sample-d.txt
  each have 30, sample-e.txt has 11 vertices.

There are output files for sample-ab.txt run with MST and FASTTSP,
while samples c-e and the spec were run with all 3 modes.

The file sample-d.txt is the same as sample-c.txt, EXCEPT
that it has been shifted up and to the right by 50, putting all
vertices in Quadrant 1 of the graph.  If you get the same answers
when running samples c and d in MST mode, and sample c output is
wrong, it's because you forgot to account for transitioning from
one region to another.  When running OPTTSP, if you get sample d
right but sample c wrong, it's because you forgot that in FAST
and OPT, you DO NOT consider the border between regions.

The file sample-e.txt has 11 vertices selected from sample d;
the vertices were selected so that most fast and optimal solutions
give different output.  This test case should help with debugging
OPTTSP mode.

SYSTEM     = x86-64_sles10_4.1
LIBFORMAT  = static_pic

# ---------------------------------------------------------------------         
# Compiler selection                                                            
# ---------------------------------------------------------------------         

CCC = g++

# ---------------------------------------------------------------------         
# Compiler options                                                              
# ---------------------------------------------------------------------         

CCOPT = -std=c++11 -m64  -fPIC -fexceptions -DNDEBUG -DIL_STD -Wall

# ---------------------------------------------------------------------         
# Link options and libraries                                                    
# ---------------------------------------------------------------------         

CCFLAGS = $(CCOPT) 
CCLNFLAGS = -lm -pthread 

#------------------------------------------------------------                   
#  make all      : to compile.                                     
#  make execute  : to compile and execute.                         
#------------------------------------------------------------    

ROUTE.exe: main.o ece556.o aStar.o computeEdgeWeights.o netOrdering.o
	/bin/rm -f ROUTE.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) main.o ece556.o aStar.o computeEdgeWeights.o netOrdering.o $(CCLNFLAGS) -o ROUTE.exe

Test_edgepts.exe: test_edgeid.o ece556.o 
	/bin/rm -f Test_edgepts.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) test_edgeid.o ece556.o $(CCLNFLAGS) -o Test_edgepts.exe

Test_multiset.exe: functorTest.o  ece556.o 
	/bin/rm -f Test_multiset.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) functorTest.o ece556.o $(CCLNFLAGS) -o Test_multiset.exe

createTest.exe: createTestFiles.o ece556.o
	/bin/rm -f createTest.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) createTestFiles.o ece556.o $(CCLNFLAGS) -o createTest.exe

createTestFiles.o: createTestFiles.cpp ece556.h
	/bin/rm -f createTestFiles.o
	$(CCC) $(CCFLAGS) createTestFiles.cpp -c

test_edgeid.o: test_edgeid.cpp ece556.h
	/bin/rm -f test_edgeid.o
	$(CCC) $(CCFLAGS) test_edgeid.cpp -c

functorTest.o: functorTest.cpp ece556.h
	/bin/rm -f functorTest.o
	$(CCC) $(CCFLAGS) functorTest.cpp -c

main.o: main.cpp ece556.h aStar.h
	/bin/rm -f main.o
	$(CCC) $(CCFLAGS) main.cpp -c

ece556.o: ece556.cpp ece556.h aStar.h
	/bin/rm -f ece556.o
	$(CCC) $(CCFLAGS) ece556.cpp -c

aStar.o: aStar.h aStar.h ece556.h
	/bin/rm -f aStar.o
	$(CCC) $(CCFLAGS) aStar.cpp -c
	
computeEdgeWeights.o: computeEdgeWeights.cpp ece556.h computeEdgeWeights.h
	/bin/rm -f computeEdgeWeights.o
	$(CCC) $(CCFLAGS) computeEdgeWeights.cpp -c
	
netOrdering.o: netOrdering.cpp ece556.h netOrdering.h
	/bin/rm -f netOrdering.o
	$(CCC) $(CCFLAGS) netOrdering.cpp -c

clean:
	/bin/rm -f *~ *.o ROUTE.exe 

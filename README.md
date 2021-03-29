# AIFA Assignment-1

A multi-agent collaborative A\* algorithm for efficient path planning of multiple Electric Vehicles.

## Getting Started

This repository is for the term project of AIFA course 2021. The code has been well commented and organized for easy understanding and use.

### Setup

All code was developed and tested on Ubuntu 20.04.6 with C++ language (Supports older versions of Ubuntu as well).

### Input Format

```
**input.txt** file contains the adjacency matrix format of the input graph
```

We are giving input for the graph in the form of Adjacency Matrix in a text file. If you want to change the input graph, you can change it in the text file.
The text file is read line by line where each line represents a row for the adjacency matrix and values are separated by space delimiter. It has to be a square matrix. If the number of rows and columns are not equal then the program terminates with an error message.

```
To change number/ parameters of Electric Vehicles, refer the instructions below.
The format is: EV(int S = 0, int D = 7, int id = vehicle_id, float B = 10, float c = 1, float d = 0.5, float M = 50, float s = 0.7)
```

If you want to change the number of EVs you can changeit in the source code.
We have already given some default values to the parameters above so even if certain parameters are not given the object will automatically initiate taccording to the default values.

### Output Format

```
4<-<-3<-2<-8 :EV 0
Time elapsed = 166 clicks.
.
.
.
Execution time for the code: 0.000785 seconds
Max time taken to travel from source to destination: 0.000178 seconds
```

The first line represents the path taken by each EV along with its id on the right hand side.
The second line represents the clock time/ clicks elapsed during its journey. Similar output is shown for each of the EVs.

The second last line shows the execution time for the code.
The last line shows the max{T_r}, i.e., the max duration of time spent by the EVs.

## Running the code

```
git clone https://github.com/AnilKumarYogi/AIFA_assignment1.git
cd AIFA_assignment1/aifa_assign1
g++ aifa_assign1.cpp
./a.out
```

## Authors

- **Sakshi Dwivedi**
- **Anil Yogi**
- **Cherry Taly**
- **Yogesh Chawla**

This example adds a database to the `4a_simple_ea_xor` example.

Naturally, look at that example first.

You learn:
- How to save intermediate and final results to a database using the SQLAlchemy ORM.
- How to read and plot the results. You could also use this knowledge to recover experiments that have crashed half-way.

Beforehand you should have:
- A basic understanding of relational databases.
- An understanding of dataclasses is beneficial.
- Basic knowledge of the Pandas library.

Introduction:
In the previous example, all results were written to the console.
We would like to store all interesting data we come across in a more structured way.
There are many options, such as saving text to files, various kinds of databases, or others such as the 'weights and biases' service.
Here we choose to use the SQLite database.
This a local relational database stored in a single file that is designed to be as simple as possible.
It is very beneficial to have a basic understanding of relational databases, as this tutorial will not explain them in-depth.

To simplify working with this database, we will use to 'object-relational mapping' (ORM) library 'SQLAlchemy'.
This software lets you declare the database structure in Python, simply by creating classes similar to Python's *dataclass*.
If you do not know what a dataclass is, it is worth looking that up.
However, it will probably also become apparent how SQLAlchemy works without that knowledge.
The main addition in this tutorial is that some classes are changed or added to follow this ORM description.
These classes can be found in the `database_components` package.
If at any point you do not understand something related to SQLAlchemy,
be aware that it is a third-party library with an extensive documentation available to you online.

Look at all files in the example carefully, and try to understand what each class is responsible for.
The main files in this tutorial are
- `main.py`, for running the experiment, and
- `plot.py`, for plotting experiment results.

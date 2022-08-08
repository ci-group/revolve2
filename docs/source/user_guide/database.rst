========
Database
========
All results and state in Revolve2 are stored using a database.
Revolve2 provides no specific interface to this database; instead this is handled by the SQLAlchemy library.
See `<https://www.sqlalchemy.org/>`_.
All databases supported by SQLAlchemy can be used, but as long as you have no specific requirements SQLite is recommended.

Processes can use the database to store their state and results.
Object that are simply data are serialized to the database using ``serializers``.
These are classes inheriting from the ``Serializer`` interface that provide serialization and deserialization of a single type of object to and from a database.
Objects serialized to a database are identified using an integer identifier.

from PyQt5.QtWidgets import (QWidget, QVBoxLayout)
import pandas as pd
from sqlalchemy import select
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ..backend_example.database_components import Experiment, Generation, Individual, Population
from revolve2.experimentation.database import OpenMethod, open_database_sqlite

class EmbeddedPlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create a layout for the widget
        layout = QVBoxLayout()
        
        # Create a matplotlib figure and canvas
        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        
        # Add canvas to layout
        layout.addWidget(self.canvas)
        self.setLayout(layout)
    
    def plot_fitness(self, database_path):
        """
        Plot fitness over generations for all experiments, averaged
        
        Args:
            database_path (str): Path to the SQLite database
        """
        # Clear previous plot
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        # Open database
        dbengine = open_database_sqlite(
            database_path, 
            open_method=OpenMethod.OPEN_IF_EXISTS
        )
        
        # Read data
        df = pd.read_sql(
            select(
                Experiment.id.label("experiment_id"),
                Generation.generation_index,
                Individual.fitness,
            )
            .join_from(Experiment, Generation, Experiment.id == Generation.experiment_id)
            .join_from(Generation, Population, Generation.population_id == Population.id)
            .join_from(Population, Individual, Population.id == Individual.population_id),
            dbengine,
        )
        
        # Aggregate data
        agg_per_experiment_per_generation = (
            df.groupby(["experiment_id", "generation_index"])
            .agg({"fitness": ["max", "mean"]})
            .reset_index()
        )
        agg_per_experiment_per_generation.columns = [
            "experiment_id",
            "generation_index",
            "max_fitness",
            "mean_fitness",
        ]
        
        agg_per_generation = (
            agg_per_experiment_per_generation.groupby("generation_index")
            .agg({"max_fitness": ["mean", "std"], "mean_fitness": ["mean", "std"]})
            .reset_index()
        )
        agg_per_generation.columns = [
            "generation_index",
            "max_fitness_mean",
            "max_fitness_std",
            "mean_fitness_mean",
            "mean_fitness_std",
        ]
        
        # Plot max fitness
        ax.plot(
            agg_per_generation["generation_index"],
            agg_per_generation["max_fitness_mean"],
            label="Max fitness",
            color="b",
        )
        ax.fill_between(
            agg_per_generation["generation_index"],
            agg_per_generation["max_fitness_mean"] - agg_per_generation["max_fitness_std"],
            agg_per_generation["max_fitness_mean"] + agg_per_generation["max_fitness_std"],
            color="b",
            alpha=0.2,
        )
        
        # Plot mean fitness
        ax.plot(
            agg_per_generation["generation_index"],
            agg_per_generation["mean_fitness_mean"],
            label="Mean fitness",
            color="r",
        )
        ax.fill_between(
            agg_per_generation["generation_index"],
            agg_per_generation["mean_fitness_mean"] - agg_per_generation["mean_fitness_std"],
            agg_per_generation["mean_fitness_mean"] + agg_per_generation["mean_fitness_std"],
            color="r",
            alpha=0.2,
        )
        
        # Customize plot
        ax.set_xlabel("Generation index")
        ax.set_ylabel("Fitness")
        ax.set_title("Mean and max fitness across repetitions with std as shade")
        ax.legend()
        
        # Adjust layout and redraw
        self.figure.tight_layout()
        self.canvas.draw()
        
        # Optionally save the figure
        self.figure.savefig(f"gui/resources/figures/{database_path.split('/')[-1]}.png")
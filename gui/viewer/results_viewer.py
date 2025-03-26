from PyQt5.QtWidgets import (QWidget, QVBoxLayout)
import pandas as pd
from sqlalchemy import select
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ..backend_example.database_components import Experiment, Generation, Individual, Population
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from PyQt5.QtCore import QTimer

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



class EmbeddedPlotWidgetDynamic(QWidget):
    def __init__(self, parent=None, update_interval=1000):
        super().__init__(parent)
        
        # Create a layout for the widget
        layout = QVBoxLayout()
        
        # Create a matplotlib figure and canvas
        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        
        # Add canvas to layout
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Store database path
        self.database_path = None
        
        # Setup timer for periodic updates
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(update_interval)  # Update every second
        
        # Flag to prevent multiple simultaneous updates
        self.is_updating = False
        self.num_generations = 10

    def set_database_path(self, database_path):
        self.database_path = database_path

    def set_num_generations(self, num_generations):
        self.num_generations = num_generations

    def update_plot(self):
        """
        Update the plot with the latest data from the database.
        """
        if self.is_updating or not self.database_path:
            return

        try:
            self.is_updating = True

            # Clear previous plot
            self.figure.clear()
            ax = self.figure.add_subplot(111)

            # Open database
            dbengine = open_database_sqlite(
                self.database_path,
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

            if df.empty or not ((df["generation_index"] == 0) & df["fitness"].notna()).any():
                print("Waiting for the first generation to complete...")
                self.draw_base_plot(ax)  # Show base plot
                return

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

            ax.set_xlim(0, self.num_generations)

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

        except Exception as e:
            print(f"Error updating plot: {e}")

        finally:
            self.is_updating = False


    def draw_base_plot(self, ax):
        """
        Draws a placeholder base plot until valid data is available.
        """
        ax.set_xlabel("Generation index")
        ax.set_ylabel("Fitness")
        ax.set_title("No data yet...")
        ax.text(0.5, 0.5, "Waiting for first generation to complete", fontsize=14, ha="center", va="center", transform=ax.transAxes)
        self.figure.tight_layout()
        self.canvas.draw()

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def extract_on_times(
    df: pd.DataFrame, channel: int, time_col: str = "Time [s]"
):
    """
    Given a dataframe with a time column and a digital signal column (0/1),
    return an array of ON-time durations in seconds.
    """

    t = df[time_col].values
    x = df[f"Channel {channel}"].values

    # Detect edges.
    rising_edges = np.where((x[:-1] == 0) & (x[1:] == 1))[0] + 1
    falling_edges = np.where((x[:-1] == 1) & (x[1:] == 0))[0] + 1

    # Align edge counts (must start with rising, end with falling).
    if len(falling_edges) == 0 or len(rising_edges) == 0:
        return np.array([])

    if falling_edges[0] < rising_edges[0]:
        falling_edges = falling_edges[1:]

    if rising_edges[-1] > falling_edges[-1]:
        rising_edges = rising_edges[:-1]

    # Compute ON durations.
    on_times = t[falling_edges] - t[rising_edges]
    return on_times


def compute_stats(on_times):
    """
    Compute summary statistics for ON-time array (seconds).
    Returned values are in *microseconds* for readability.
    """

    if len(on_times) == 0:
        return {"error": "No valid pulses detected."}

    stats = {
        "count": len(on_times),
        "mean_us": float(np.mean(on_times) * 1e6),
        "median_us": float(np.median(on_times) * 1e6),
        "min_us": float(np.min(on_times) * 1e6),
        "max_us": float(np.max(on_times) * 1e6),
        "std_us": float(np.std(on_times) * 1e6),
    }

    return stats


def plot_on_times(on_times, bins: int = 50):
    """
    Plot histogram and boxplot of ON durations using matplotlib.
    """

    if len(on_times) == 0:
        print("No pulses to plot.")
        return

    on_us = on_times * 1e6  # Convert to microseconds.

    plt.figure(figsize=(12, 5))

    # Histogram.
    plt.subplot(1, 2, 1)
    plt.hist(on_us, bins=bins)
    plt.xlabel("Pulse Width (us)")
    plt.ylabel("Count")
    plt.title("ON-Time Histogram")

    # Boxplot.
    plt.subplot(1, 2, 2)
    plt.boxplot(on_us, vert=True)
    plt.ylabel("Pulse Width (us)")
    plt.title("ON-Time Boxplot")

    plt.tight_layout()
    plt.show()


def main(input_file: str, channel: int):
    try:
        df = pd.read_csv(input_file)
    except FileNotFoundError:
        print(f"Error: Could not find '{input_file}'.")
        return

    print(f"Loaded {len(df)} samples from {input_file}")
    print(f"Analyzing Channel {channel}...")

    # Extract ON pulse durations.
    on_times = extract_on_times(df, channel=channel)

    if len(on_times) == 0:
        print("No valid ON pulses detected.")
        return

    # Compute statistics.
    stats = compute_stats(on_times)

    print("\n=== Pulse Statistics (microseconds) ===")
    for key, value in stats.items():
        print(f"{key}: {value}")

    # Plot histogram + boxplot.
    plot_on_times(on_times)


if __name__ == "__main__":
    main("single_cycle.csv", 7)

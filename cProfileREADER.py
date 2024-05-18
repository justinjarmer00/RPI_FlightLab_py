import pstats

# Create a Stats object
stats = pstats.Stats('profile_output1.txt')

# Sort and print the statistics
# You can sort by 'cumulative', 'time', 'calls', etc.
stats.sort_stats('cumulative').print_stats(100)  # Print top 10 functions
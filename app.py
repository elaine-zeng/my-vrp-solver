import streamlit as st
import pandas as pd
from VRPSolver import VRPSolver # Assuming VRPSolver class is in VRPSolver.py
import time
import io
import folium
import streamlit_folium as st_folium

# Set page config for a wider layout
st.set_page_config(layout="wide", page_title="VRP Solver for Non-profits")

# --- Initialize VRPSolver (using Streamlit's session state to persist) ---
if 'solver' not in st.session_state:
    st.session_state.solver = VRPSolver()

solver = st.session_state.solver

st.title("🚙 Open Route Optimizer")
st.markdown("""
Welcome! This tool helps optimize routes to save time and resources.
Enter your addresses or upload a text file, specify the number of vehicles, and get optimized routes.
""")

st.warning("""
**Important Notes on Free Usage:**
* This tool uses **free public APIs** (Nominatim for geocoding, OSRM for routing). These APIs have **rate limits**. If you submit too many addresses or run the solver too often,
    you might experience delays or temporary errors.
* For best results, use **less than 50-70 addresses** at a time, especially with the OSRM public server. For **very large problems or consistent high usage**, consider self-hosting Nominatim/OSRM or using paid services.
""")

st.markdown("""
**When entering addresses, the first address is the depot by default.**
""")

st.markdown("""
The VRP Tool solves to miminize the maximum distance traveled by a vehicle while using all available vehicles.
""")

# --- Input Section ---
input_method = st.radio("Choose input method:", ("Enter addresses manually", "Upload addresses from text file"))

addresses_input_string = "" # Renamed variable to avoid conflict with `addresses_input` later
uploaded_file = None

if input_method == "Enter addresses manually":
    st.markdown("Enter one address per line.")
    addresses_input_string = st.text_area(
        "Addresses (e.g., '123 Main St, City, State'):",
        height=200,
        placeholder="712 3rd St, West Lafayette, IN 47907\n101 N Grant St, West Lafayette, IN 47906\n..."
    )
else: # Upload from text file
    uploaded_file = st.file_uploader("Upload a text file with one address per line", type=["txt"])
    if uploaded_file is not None:
        # --- FIX APPLIED HERE ---
        addresses_input_string = uploaded_file.read().decode("utf-8")
        # --- END FIX ---

# --- Process addresses from either input method ---
# Get list of non-empty, stripped addresses
current_addresses_list = [addr.strip() for addr in addresses_input_string.strip().split('\n') if addr.strip()]
count_valid_addresses = len(current_addresses_list)

# --- Calculate slider max_value based on input ---
slider_max_value = max(2, min(10, count_valid_addresses-1)) # Cap at 10, but ensure at least 2 for slider

# Initial value for the slider
initial_slider_value = min(1, slider_max_value) # Start at 1, but don't exceed slider_max_value

num_vehicles = st.slider(
    "Number of Vehicles:",
    min_value=1,
    max_value=slider_max_value,
    value=initial_slider_value # Ensure default value is within the new min/max range
)

# After the slider, if count_valid_addresses is 0 or 1, force num_vehicles to 1
# This handles the logic even if the slider itself showed higher values for display purposes
if count_valid_addresses <= 1:
    num_vehicles = 1
    if count_valid_addresses == 0:
        st.info("Please enter at least one address.")
    else: # count_valid_addresses == 1 (only depot)
        st.info("With only one address (depot), only one vehicle is needed. Set to 1 vehicle.")

# --- Solve Button ---
if st.button("Optimize Routes"):
    if count_valid_addresses < 2: # At least 2 addresses (depot + 1 stop) needed for routing
        st.error("Please enter at least two unique addresses (including your depot) to create a route.")
        st.stop()
    
    # Prepare addresses
    raw_addresses = [addr.strip() for addr in addresses_input_string.strip().split('\n') if addr.strip()]
    
    # Ensure depot (first address) is always included and valid
    depot_address = raw_addresses[0]
    other_addresses = raw_addresses[1:]

    # To handle potential duplicates and ensure depot is unique and first
    unique_addresses = [depot_address]
    for addr in other_addresses:
        if addr not in unique_addresses:
            unique_addresses.append(addr)

    if len(unique_addresses) < 2:
        st.error("Please provide at least two unique addresses (including the depot) to create a route.")
        st.stop()
    
    # Use a spinner for long operations
    with st.spinner("Geocoding addresses... This may take a while due to API rate limits."):
        # Call geocode_addresses from the solver instance
        coordinates, failed_geocodes = solver.geocode_addresses(unique_addresses)
        if failed_geocodes:
            st.warning(f"Could not geocode the following addresses: {', '.join(failed_geocodes)}. They will be excluded from routing.")
        if not coordinates:
            st.error("No addresses could be geocoded. Please check your input.")
            st.stop()
        
        # The solver's internal `self.addresses` and `self.coordinates` are already updated by `geocode_addresses`
        # with only the successfully geocoded points. So no need to re-assign `solver.addresses` or `solver.coordinates` here.
    
    # After geocoding, check if enough valid addresses remain for routing
    if len(solver.coordinates) < 2:
        st.error("After geocoding, fewer than two valid addresses remain. Cannot create routes.")
        st.stop()
    
    # Create distance matrix with progress updates
    matrix_placeholder = st.empty() # Placeholder to update progress messages
    distance_matrix = None
    with st.spinner("Calculating road distances (this can be slow for many addresses due to API limits)..."):
        # The `create_distance_matrix` method in VRPSolver is a generator,
        # so we iterate over it to display progress.
        for progress_msg in solver.create_distance_matrix():
            matrix_placeholder.text(progress_msg)
        distance_matrix = solver.distance_matrix # Get the final matrix after generation

    if distance_matrix is None:
        st.error("Failed to create distance matrix. Cannot solve VRP.")
        st.stop()

    # Solve VRP
    st.subheader("Solving VRP...")
    with st.spinner("Optimizing routes..."):
        # The depot index is always 0 based on your current logic (first address is depot)
        routes = solver.solve_vrp(num_vehicles=num_vehicles, depot_index=0) 

    if routes:
        st.success("Routes optimized successfully!")

        # --- Display Results ---
        st.subheader("Solution Summary")
        summary_df = solver.get_route_summary() # Use the new method name
        st.dataframe(summary_df)

        st.subheader("Route Maps")
        route_map = solver.create_route_map()
        if route_map:
            st.markdown("Explore the interactive map below. Click on markers for details.")
            st_folium.folium_static(route_map, width=1000, height=600) # Adjust width/height as needed
        else:
            st.warning("Could not generate map. This might happen if no routes were found or all geocoding failed.")
        
        st.subheader("Google Maps Directions Links")
        st.markdown("Click on a link below to open directions in Google Maps:")
        
        # Filter routes to only display active ones
        active_routes = [r for r in routes if len(r['route_node_indices']) > 2]

        if active_routes:
            for route_info in active_routes:
                google_link = solver.generate_Maps_link(route_info)
                # Use the addresses list from the route_info itself, as it's already filtered/ordered
                # For display, only show the customer stops, not the start/end depot.
                display_addresses = ' -> '.join(route_info['addresses'][1:-1]) # Exclude first and last (depot)
                
                st.markdown(f"**Vehicle {route_info['vehicle_id']}** ({len(route_info['route_node_indices']) - 2} stops, {route_info['distance_m']/1000:.2f} km): [Open in Google Maps]({google_link})")
                st.markdown(f"*{display_addresses}*")
        else:
            st.info("No active routes were generated. All vehicles might be unused or only depot-to-depot routes were found.")

    else:
        st.error("Could not find a solution for the given parameters. This could be due to too few addresses, issues with geocoding/routing APIs, or complex constraints.")
        st.info("Try reducing the number of addresses or vehicles, or ensure all addresses are valid.")


# --- How to Run This App ---
st.sidebar.markdown(
        """
    <a href='https://ko-fi.com/elainezeng' target='_blank'><img height='50' style='border:0px;height:50px;' 
    src='https://storage.ko-fi.com/cdn/kofi5.png?v=3' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>        
    """,
        unsafe_allow_html=True,
    )
st.sidebar.header("How to Use")
st.sidebar.markdown("""
1.  **Input Addresses:** Type them manually (one per line) or upload a `.txt` file.
    * The **first address** you provide will be treated as your **depot** (the start and end point for all vehicles).
2.  **Set Vehicles:** Use the slider to choose the number of vehicles available for your fleet.
3.  **Optimize Routes:** Click the "Optimize Routes" button to start the calculation.
4.  **View Results:** After optimization, you'll see a summary table, an interactive map, and direct links to Google Maps for each route.
""")
st.sidebar.info("Developed for non-profits using open-source tools (Nominatim, OSRM, OR-Tools).")

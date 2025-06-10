"""
Base Plotly Module for Delta Robot Visualizations
Provides standardized 3D plotting with coordinate system and consistent styling
FIXED: 1:1:1 aspect ratio for accurate spatial representation
"""

import plotly.graph_objects as go
import numpy as np

def add_coordinate_system(fig, scale=10.0, origin=(0, 0, 0)):
    """Add XYZ coordinate system at specified origin"""
    
    x0, y0, z0 = origin
    
    # X-axis (red)
    fig.add_trace(go.Scatter3d(
        x=[x0, x0 + scale],
        y=[y0, y0],
        z=[z0, z0],
        mode='lines+text',
        line=dict(color='red', width=4),
        text=['', 'X'],
        textposition='middle right',
        name='X-axis',
        showlegend=False
    ))
    
    # Y-axis (green)
    fig.add_trace(go.Scatter3d(
        x=[x0, x0],
        y=[y0, y0 + scale],
        z=[z0, z0],
        mode='lines+text',
        line=dict(color='green', width=4),
        text=['', 'Y'],
        textposition='middle right',
        name='Y-axis',
        showlegend=False
    ))
    
    # Z-axis (blue)
    fig.add_trace(go.Scatter3d(
        x=[x0, x0],
        y=[y0, y0],
        z=[z0, z0 + scale],
        mode='lines+text',
        line=dict(color='blue', width=4),
        text=['', 'Z'],
        textposition='middle right',
        name='Z-axis',
        showlegend=False
    ))
    
    # Origin point
    fig.add_trace(go.Scatter3d(
        x=[x0],
        y=[y0],
        z=[z0],
        mode='markers',
        marker=dict(size=6, color='black'),
        name='Origin',
        showlegend=False
    ))

def create_base_3d_figure(title="Delta Robot Visualization", width=1600, height=1200):
    """
    Create standardized 3D figure with coordinate system and consistent styling
    FIXED: Enforces 1:1:1 aspect ratio for accurate spatial representation
    
    Args:
        title: Plot title
        width, height: Figure dimensions
        
    Returns:
        Plotly Figure object ready for additional traces
    """
    
    fig = go.Figure()
    
    # Add coordinate system at origin
    add_coordinate_system(fig, scale=15.0)
    
    # Standard layout with FIXED 1:1:1 aspect ratio
    fig.update_layout(
        title=dict(
            text=title,
            x=0.5,
            font=dict(size=16)
        ),
        scene=dict(
            xaxis_title="X (mm)",
            yaxis_title="Y (mm)", 
            zaxis_title="Z (mm)",
            aspectmode='data',  # Respect data ranges but maintain equal scaling
            aspectratio=dict(x=1, y=1, z=1),  # Equal scaling per unit
            xaxis=dict(dtick=50),  # Same step size for all axes
            yaxis=dict(dtick=50),
            zaxis=dict(dtick=50),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)  # Nice 3D viewing angle
            )
        ),
        showlegend=True,
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="rgba(0,0,0,0.2)",
            borderwidth=1
        ),
        width=width,
        height=height,
        margin=dict(l=0, r=0, t=40, b=0)
    )
    
    return fig

def add_robot_base_circle(fig, radius, color='lightgray', name='Base Circle'):
    """Add robot base circle for reference"""
    
    theta = np.linspace(0, 2*np.pi, 100)
    x_circle = radius * np.cos(theta)
    y_circle = radius * np.sin(theta)
    z_circle = np.zeros_like(theta)
    
    fig.add_trace(go.Scatter3d(
        x=x_circle,
        y=y_circle,
        z=z_circle,
        mode='lines',
        line=dict(color=color, width=2, dash='dot'),
        name=name,
        showlegend=False
    ))

def add_vector_arrow(fig, start, end, color='orange', name='Vector', width=5):
    """Add a vector arrow from start to end point"""
    
    fig.add_trace(go.Scatter3d(
        x=[start[0], end[0]],
        y=[start[1], end[1]],
        z=[start[2], end[2]],
        mode='lines+markers',
        line=dict(color=color, width=width),
        marker=dict(
            size=[4, 8], 
            color=[color, color],
            symbol=['circle', 'diamond']
        ),
        name=name
    ))

# Standard color scheme for robot components
ROBOT_COLORS = {
    'actuator_A': 'red',
    'actuator_B': 'green', 
    'actuator_C': 'blue',
    'fermat_point': 'purple',
    'end_effector': 'orange',
    'input_vector': 'darkorange',
    'base_positions': 'gray',
    # Segment-specific colors
    'joint_chain': 'lightblue',
    'joint_points': 'lightblue',
    'segment_points': 'purple',
    'segment_chain': 'purple',
    'segment_directions': ['rgb(205, 150, 200)', 'rgb(230, 175, 225)', 'rgb(180, 100, 175)', 'rgb(255, 200, 250)', 'rgb(160, 80, 155)'],
    'previous_direction': 'darkgray'
}
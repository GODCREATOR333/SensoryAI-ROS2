            try:
                self.get_logger().info(f'Processing frame: format={frame.format.name}, width={frame.width}, height={frame.height}')
                
                # Extract raw data from each plane using the correct method
                y_plane = np.frombuffer(frame.planes[0].buffer, np.uint8).reshape((frame.planes[0].height, frame.planes[0].line_size))[:, :frame.width]
                u_plane = np.frombuffer(frame.planes[1].buffer, np.uint8).reshape((frame.planes[1].height, frame.planes[1].line_size))[:, :frame.width // 2]
                v_plane = np.frombuffer(frame.planes[2].buffer, np.uint8).reshape((frame.planes[2].height, frame.planes[2].line_size))[:, :frame.width // 2]
                
                self.get_logger().info(f'Reshaped planes - Y: {y_plane.shape}, U: {u_plane.shape}, V: {v_plane.shape}')
                
                # Upsample the U and V planes to match Y plane size
                u_upsampled = np.repeat(np.repeat(u_plane, 2, axis=0), 2, axis=1)
                v_upsampled = np.repeat(np.repeat(v_plane, 2, axis=0), 2, axis=1)
                
                # Ensure all planes have the same shape
                y_plane = y_plane[:frame.height, :frame.width]
                u_upsampled = u_upsampled[:frame.height, :frame.width]
                v_upsampled = v_upsampled[:frame.height, :frame.width]
                
                # Stack the planes to create a YUV frame
                yuv_frame = np.stack([y_plane, u_upsampled, v_upsampled], axis=-1)
                
                self.get_logger().info(f'Final YUV frame shape: {yuv_frame.shape}')
                
                # Convert YUV to BGR for display
                bgr_frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR)
                
                # Display the frame
                cv2.imshow('Received Frame', bgr_frame)
                cv2.waitKey(1)  # Wait for 1ms to update the window
                
                # Log YUV420P format information
                self.get_logger().info(f'YUV420P frame info:')
                self.get_logger().info(f'  Y plane: shape={y_plane.shape}, dtype={y_plane.dtype}, min={y_plane.min()}, max={y_plane.max()}')
                self.get_logger().info(f'  U plane: shape={u_plane.shape}, dtype={u_plane.dtype}, min={u_plane.min()}, max={u_plane.max()}')
                self.get_logger().info(f'  V plane: shape={v_plane.shape}, dtype={v_plane.dtype}, min={v_plane.min()}, max={v_plane.max()}')
            
            except Exception as e:
                self.get_logger().error(f'Error processing video frame: {str(e)}')
                import traceback
                self.get_logger().error(f'Traceback: {traceback.format_exc()}')
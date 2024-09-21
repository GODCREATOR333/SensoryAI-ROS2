        try:
            if self.video_writer is None:
                # Initialize video writer once we receive the first frame
                self.frame_width = frame.width
                self.frame_height = frame.height
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(self.output_file, fourcc, self.fps,
                                                    (self.frame_width, self.frame_height))
                self.get_logger().info(f'VideoWriter initialized: {self.output_file}')

            # Convert YUV to RGB
            img_rgb = frame.to_ndarray(format='rgb24')
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            # Run YOLOv8 object detection
            results = self.model(img_rgb)  # Perform detection

            # Process results and draw bounding boxes and labels
            for result in results:
                boxes = result.boxes.xyxy  # Get bounding boxes
                class_ids = result.boxes.cls  # Get class IDs
                for box, class_id in zip(boxes, class_ids):
                    x1, y1, x2, y2 = map(int, box[:4])
                    label = f'Class {int(class_id)}'  # You can replace this with actual class names if you have a mapping

                    cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
                    cv2.putText(img_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Draw label

            # Calculate FPS
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.previous_frame_time >= 1.0:  # Every second
                fps = self.frame_count
                self.frame_count = 0
                self.previous_frame_time = current_time
            else:
                fps = self.frame_count / (current_time - self.previous_frame_time)

            # Draw FPS on the frame
            cv2.putText(img_bgr, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # Write the frame to disk
            self.video_writer.write(img_bgr)
            self.get_logger().info(f'Frame written to disk: {self.output_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing video frame: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
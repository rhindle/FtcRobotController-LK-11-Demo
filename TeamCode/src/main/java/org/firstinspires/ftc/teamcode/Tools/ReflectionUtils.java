package org.firstinspires.ftc.teamcode.Tools;

import java.lang.reflect.Field;

public class ReflectionUtils {
   public static Field getFieldX(Class clazz, String fieldName) {
      try {
         Field f = clazz.getDeclaredField(fieldName);
         f.setAccessible(true);
         return f;
      } catch (NoSuchFieldException e) {
         Class superClass = clazz.getSuperclass();
         if (superClass != null) {
            return getFieldX(superClass, fieldName);
         }
      }
      return null;
   }
}
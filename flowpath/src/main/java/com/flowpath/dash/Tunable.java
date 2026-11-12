package com.flowpath.dash;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Tandai class (di file mana pun) agar static variable-nya otomatis muncul di dashboard lewat Dash.scan(...). */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Tunable {}
